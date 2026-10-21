// Isolated RK3568 evaluation: identical decoded crops, canonical NCHW heads.
#include <rknn_api.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static void check(int code,const char* step) {
  if(code!=RKNN_SUCC)throw std::runtime_error(std::string(step)+" failed: "+std::to_string(code));
}
struct Context {
  rknn_context value=0;
  ~Context(){if(value)rknn_destroy(value);}
};
int main(int argc,char** argv) {
  if(argc!=4){std::cerr<<"usage: probe MODEL LIST OUTPUT_PREFIX\n";return 1;}
  try {
    cv::setNumThreads(1);
    std::ifstream file(argv[1],std::ios::binary|std::ios::ate);
    if(!file)throw std::runtime_error("Missing model");
    const auto size=file.tellg();file.seekg(0);
    std::vector<unsigned char> model(static_cast<size_t>(size));
    file.read(reinterpret_cast<char*>(model.data()),model.size());
    if(!file)throw std::runtime_error("Incomplete model");
    Context context;
    check(rknn_init(&context.value,model.data(),model.size(),0,nullptr),"init");
    model.clear();model.shrink_to_fit();
    rknn_input_output_num count{};check(rknn_query(context.value,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count)),"query count");
    if(count.n_input!=1 || count.n_output!=4)throw std::runtime_error("Unexpected input/output count");
    rknn_tensor_attr input_attr{};
    check(rknn_query(context.value,RKNN_QUERY_INPUT_ATTR,&input_attr,sizeof(input_attr)),"query input");
    const bool nhwc=input_attr.fmt==RKNN_TENSOR_NHWC;
    if(input_attr.n_dims!=4 || input_attr.dims[0]!=1 ||
       input_attr.dims[nhwc?1:2]!=320 || input_attr.dims[nhwc?2:3]!=1600 ||
       input_attr.dims[nhwc?3:1]!=3)throw std::runtime_error("Expected 1600x320 RGB input");
    const std::array<std::array<unsigned,4>,4> shapes{{{{1,200,72,4}},{{1,100,81,4}},{{1,2,72,4}},{{1,2,81,4}}}};
    std::array<int,4> mapping{{-1,-1,-1,-1}};
    std::array<rknn_tensor_attr,4> attrs{};
    for(unsigned i=0;i<4;++i) {
      attrs[i].index=i;
      check(rknn_query(context.value,RKNN_QUERY_OUTPUT_ATTR,&attrs[i],sizeof(attrs[i])),"query output");
      for(unsigned head=0;head<4;++head) {
        bool matches=attrs[i].n_dims==4;
        for(unsigned dim=0;dim<4 && matches;++dim)matches=attrs[i].dims[dim]==shapes[head][dim];
        if(matches) {
          if(mapping[head]!=-1)throw std::runtime_error("Duplicate output schema");
          mapping[head]=i;
        }
      }
    }
    for(int index:mapping)if(index<0)throw std::runtime_error("Canonical NCHW schema not found");
    std::ifstream list(argv[2]);if(!list)throw std::runtime_error("Missing input list");
    std::vector<std::string> paths;std::string path;
    while(std::getline(list,path))if(!path.empty())paths.push_back(path);
    if(paths.empty())throw std::runtime_error("Empty input list");
    std::ofstream timings(std::string(argv[3])+".timings.csv");
    timings<<"index,inference_inputs_run_outputs_ms\n";
    std::vector<double> times;
    for(size_t index=0;index<paths.size();++index) {
      cv::Mat frame=cv::imread(paths[index]);
      if(frame.rows!=320 || frame.cols!=1600 || frame.type()!=CV_8UC3)
        throw std::runtime_error("Input must be an exact 1600x320 crop");
      cv::cvtColor(frame,frame,cv::COLOR_BGR2RGB);
      rknn_input input{};input.index=0;input.type=RKNN_TENSOR_UINT8;input.fmt=RKNN_TENSOR_NHWC;
      input.size=frame.total()*frame.elemSize();input.buf=frame.data;
      const int repeats=index==0?3:1;
      for(int repeat=0;repeat<repeats;++repeat) {
        const auto started=std::chrono::steady_clock::now();
        check(rknn_inputs_set(context.value,1,&input),"inputs");
        check(rknn_run(context.value,nullptr),"run");
        std::array<rknn_output,4> outputs{};
        for(unsigned i=0;i<4;++i){outputs[i].index=i;outputs[i].want_float=1;}
        check(rknn_outputs_get(context.value,4,outputs.data(),nullptr),"outputs");
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        bool good=true;
        for(unsigned head=0;head<4;++head) {
          const auto& output=outputs[mapping[head]];
          const auto expected=shapes[head][1]*shapes[head][2]*shapes[head][3];
          const float* values=static_cast<const float*>(output.buf);
          if(!values || output.size!=expected*sizeof(float)){good=false;continue;}
          for(unsigned k=0;k<expected;++k)if(!std::isfinite(values[k])){good=false;break;}
          if(repeat==repeats-1) {
            const auto name=std::string(argv[3])+"_"+std::to_string(index)+".output"+std::to_string(head)+".f32";
            std::ofstream dump(name,std::ios::binary);
            dump.write(reinterpret_cast<const char*>(values),output.size);good=good&&bool(dump);
          }
        }
        rknn_outputs_release(context.value,4,outputs.data());
        if(!good)throw std::runtime_error("Invalid output or write failure");
        if(repeat==repeats-1){times.push_back(ms);timings<<index<<','<<ms<<'\n';}
      }
    }
    if(!timings)throw std::runtime_error("Cannot write timings");
    double sum=0;for(double ms:times)sum+=ms;
    std::sort(times.begin(),times.end());
    std::cout<<"BATCH_PROBE_PASS samples="<<times.size()<<" mean_ms="<<sum/times.size()
      <<" p50_ms="<<times[times.size()/2]<<" p95_ms="<<times[static_cast<size_t>((times.size()-1)*.95)]<<'\n';
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
