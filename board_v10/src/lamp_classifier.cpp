#include "adas/lamp_classifier.hpp"
#include "adas/npu_run_guard.hpp"
#include "adas/model_file.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace adas {
LampClassifier::LampClassifier(const std::string& path) {
  std::string problem;
  auto model = read_model_file(path, &problem);
  if (model.empty()) { error_ = "Cannot read MO FP16: " + problem; return; }
  if (rknn_init(&context_, model.data(), uint32_t(model.size()), 0, nullptr) != RKNN_SUCC) {
    context_ = 0; error_ = "MO FP16 rknn_init failed"; return;
  }
  rknn_input_output_num count{};
  rknn_tensor_attr input{}, output{};
  if (rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)) != RKNN_SUCC ||
      count.n_input != 1 || count.n_output != 1 ||
      rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input, sizeof(input)) != RKNN_SUCC ||
      rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output, sizeof(output)) != RKNN_SUCC ||
      input.n_elems != 128 * 128 * 3 || output.n_elems != 9) {
    error_ = "MO must have RGB128 input and nine logits";
  }
}
LampClassifier::~LampClassifier() { if (context_) rknn_destroy(context_); }

bool LampClassifier::predict(const cv::Mat& crop, int& category, float& confidence, float& margin) {
  if (!ready() || crop.empty()) return false;
  // MO 输入尺寸为 128×128：等比缩放并补黑边，保留箭头形状。
  // 这是网络输入尺寸；原图裁剪范围由 classify() 单独计算。
  const float scale = std::min(128.f/crop.cols,128.f/crop.rows);
  int w=std::max(1,int(std::round(crop.cols*scale))), h=std::max(1,int(std::round(crop.rows*scale)));
  cv::Mat resized, canvas(128,128,CV_8UC3,cv::Scalar(0,0,0));
  cv::resize(crop,resized,cv::Size(w,h),0,0,cv::INTER_LINEAR);
  resized.copyTo(canvas(cv::Rect((128-w)/2,(128-h)/2,w,h)));
  cv::cvtColor(canvas,canvas,cv::COLOR_BGR2RGB);
  rknn_input input{}; input.type=RKNN_TENSOR_UINT8; input.fmt=RKNN_TENSOR_NHWC;
  input.size=128*128*3; input.buf=canvas.data; input.pass_through=0;
  NpuRunGuard npu_guard(npu_run_mutex());
  if (rknn_inputs_set(context_,1,&input)!=RKNN_SUCC || rknn_run(context_,nullptr)!=RKNN_SUCC) return false;
  rknn_output output{}; output.want_float=1;
  if (rknn_outputs_get(context_,1,&output,nullptr)!=RKNN_SUCC) return false;
  if (!output.buf || output.size < 9*sizeof(float)) {
    rknn_outputs_release(context_,1,&output); return false;
  }
  const float* logits=static_cast<const float*>(output.buf);
  npu_guard.unlock();
  float peak=*std::max_element(logits,logits+9), total=0.f, probabilities[9];
  for(int i=0;i<9;++i) { probabilities[i]=std::exp(logits[i]-peak); total+=probabilities[i]; }
  category=int(std::max_element(probabilities,probabilities+9)-probabilities);
  confidence=probabilities[category]/total;
  float second=0.f;
  for(int i=0;i<9;++i) if(i!=category) second=std::max(second,probabilities[i]/total);
  margin=confidence-second;
  rknn_outputs_release(context_,1,&output);
  return std::isfinite(confidence) && std::isfinite(margin);
}

std::vector<Detection> LampClassifier::classify(const cv::Mat& bgr,const Detection& housing) {
  std::vector<Detection> accepted;
  const cv::Rect bounds(0,0,bgr.cols,bgr.rows);
  const cv::Rect roi=cv::Rect(housing.box)&bounds;
  if(roi.width<5 || roi.height<5) return accepted;
  // YOLO 定位灯组后，以 HSV 亮色连通域提出单个亮灯候选；最多分类 4 个。
  // HSV 只提供候选和颜色校验，最终灯形/方向仍由 MO 分类决定。
  cv::Mat hsv, red, green, mask, labels, stats, centers;
  cv::cvtColor(bgr(roi),hsv,cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;cv::split(hsv,channels);
  double peak_value=0;cv::minMaxLoc(channels[2],nullptr,&peak_value);
  const double brightness=std::max(90.,std::min(180.,peak_value*.65));
  cv::Mat red2;
  cv::inRange(hsv,cv::Scalar(0,100,brightness),cv::Scalar(12,255,255),red);
  cv::inRange(hsv,cv::Scalar(160,100,brightness),cv::Scalar(179,255,255),red2);
  cv::bitwise_or(red,red2,red);
  cv::inRange(hsv,cv::Scalar(35,100,brightness),cv::Scalar(100,255,255),green);
  cv::bitwise_or(red,green,mask);
  if(std::min(roi.width,roi.height)>=9)
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::Mat::ones(3,3,CV_8U));
  const int count=cv::connectedComponentsWithStats(mask,labels,stats,centers,8,CV_32S);
  std::vector<cv::Rect> lamps;
  for(int i=1;i<count;++i) {
    int x=stats.at<int>(i,cv::CC_STAT_LEFT),y=stats.at<int>(i,cv::CC_STAT_TOP);
    int w=stats.at<int>(i,cv::CC_STAT_WIDTH),h=stats.at<int>(i,cv::CC_STAT_HEIGHT);
    if(stats.at<int>(i,cv::CC_STAT_AREA)<4 || std::min(w,h)<5 || std::max(w,h)>4*std::min(w,h)) continue;
    lamps.emplace_back(roi.x+x,roi.y+y,w,h);
  }
  std::sort(lamps.begin(),lamps.end(),[](const cv::Rect& a,const cv::Rect& b){return a.area()>b.area();});
  if(lamps.size()>4) lamps.resize(4);
  if(lamps.empty()) lamps.push_back(roi);
  static const char* names[]={"红色圆灯","红色左转箭头","红色右转箭头","红色直行箭头",
                            "绿色圆灯","绿色左转箭头","绿色右转箭头","绿色直行箭头"};
  for(const cv::Rect& lamp:lamps) {
    // 显示框仍是实际亮灯区域；MO 使用外扩 1.2 倍的方形上下文裁剪。
    // 外扩保留灯泡和壳体边缘，裁剪边界始终限制在原图内。
    const int side=std::max(1,int(std::ceil(std::max(lamp.width,lamp.height)*1.2f)));
    const int cx=lamp.x+lamp.width/2,cy=lamp.y+lamp.height/2;
    const cv::Rect crop=cv::Rect(cx-side/2,cy-side/2,side,side)&bounds;
    int category=-1;float confidence=0.f,margin=0.f;
    // 第 9 类（ID 8）是拒识；最低概率 0.60，第一/第二名差值至少 0.15。
    // 模型拒识或证据不足时，不强行把候选归成红灯或绿灯。
    if(!predict(bgr(crop),category,confidence,margin) || category>=8 ||
       confidence<0.6f || margin<0.15f) continue;
    const cv::Rect local(lamp.x-roi.x,lamp.y-roi.y,lamp.width,lamp.height);
    const int red_pixels=cv::countNonZero(red(local)),green_pixels=cv::countNonZero(green(local));
    const int matching=category<4 ? red_pixels : green_pixels;
    const int opposite=category<4 ? green_pixels : red_pixels;
    // 颜色证据只否决明显冲突的分类，不会强制修改 MO 的类别。
    if(matching<3 || matching<opposite) continue;
    Detection detection=housing;
    detection.class_id=7+category;
    detection.score=confidence;
    detection.box=cv::Rect2f(lamp);
    std::ostringstream label;
    label<<names[category]<<" MO "<<int(std::round(confidence*100))<<'%';
    detection.name=label.str();
    accepted.push_back(detection);
  }
  return accepted;
}
}
