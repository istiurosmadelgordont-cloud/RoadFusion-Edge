#include <rknn_api.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

std::vector<unsigned char> read_file(const std::string& path) {
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input) return {};
  input.seekg(0, std::ios::end);
  const std::streamsize size = input.tellg();
  input.seekg(0, std::ios::beg);
  std::vector<unsigned char> data(static_cast<size_t>(size));
  input.read(reinterpret_cast<char*>(data.data()), size);
  return data;
}

const char* tensor_type(rknn_tensor_type type) {
  switch (type) {
    case RKNN_TENSOR_FLOAT32: return "float32";
    case RKNN_TENSOR_FLOAT16: return "float16";
    case RKNN_TENSOR_INT8: return "int8";
    case RKNN_TENSOR_UINT8: return "uint8";
    default: return "other";
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: ufld_benchmark MODEL.rknn IMAGE [RUNS]\n";
    return 1;
  }
  const int runs = argc > 3 ? std::max(1, std::atoi(argv[3])) : 20;
  const std::vector<unsigned char> model = read_file(argv[1]);
  if (model.empty()) {
    std::cerr << "cannot read model\n";
    return 2;
  }
  rknn_context context = 0;
  const auto load_start = std::chrono::steady_clock::now();
  const int init_code = rknn_init(&context, const_cast<unsigned char*>(model.data()),
                                  static_cast<uint32_t>(model.size()), 0, nullptr);
  const double load_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - load_start).count();
  if (init_code != RKNN_SUCC) {
    std::cerr << "rknn_init failed " << init_code << " load_ms=" << load_ms << '\n';
    return 3;
  }

  rknn_input_output_num io{};
  rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io));
  std::cout << "model_bytes=" << model.size() << " load_ms=" << load_ms
            << " inputs=" << io.n_input << " outputs=" << io.n_output << '\n';
  rknn_tensor_attr input_attr{};
  input_attr.index = 0;
  rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr));
  std::cout << "input[0] name=" << input_attr.name
            << " type=" << tensor_type(input_attr.type) << " dims=";
  for (uint32_t d = 0; d < input_attr.n_dims; ++d) std::cout << input_attr.dims[d] << 'x';
  std::cout << " fmt=" << (input_attr.fmt == RKNN_TENSOR_NHWC ? "NHWC" : "NCHW") << '\n';
  for (uint32_t i = 0; i < io.n_output; ++i) {
    rknn_tensor_attr attr{};
    attr.index = i;
    rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
    std::cout << "output[" << i << "] name=" << attr.name
              << " type=" << tensor_type(attr.type) << " dims=";
    for (uint32_t d = 0; d < attr.n_dims; ++d) std::cout << attr.dims[d] << 'x';
    std::cout << " scale=" << attr.scale << " zp=" << attr.zp << '\n';
  }

  cv::Mat frame = cv::imread(argv[2]);
  if (frame.empty()) {
    std::cerr << "cannot read image\n";
    rknn_destroy(context);
    return 4;
  }
  const int input_height = input_attr.fmt == RKNN_TENSOR_NHWC
      ? static_cast<int>(input_attr.dims[1]) : static_cast<int>(input_attr.dims[2]);
  const int input_width = input_attr.fmt == RKNN_TENSOR_NHWC
      ? static_cast<int>(input_attr.dims[2]) : static_cast<int>(input_attr.dims[3]);
  cv::resize(frame, frame, cv::Size(input_width, input_height), 0, 0, cv::INTER_LINEAR);
  cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.fmt = RKNN_TENSOR_NHWC;
  input.size = static_cast<uint32_t>(frame.total() * frame.elemSize());
  input.buf = frame.data;
  input.pass_through = 0;
  if (rknn_inputs_set(context, 1, &input) != RKNN_SUCC) {
    std::cerr << "inputs_set failed\n";
    rknn_destroy(context);
    return 5;
  }

  std::vector<double> times;
  for (int iteration = 0; iteration < runs + 2; ++iteration) {
    const auto start = std::chrono::steady_clock::now();
    const int code = rknn_run(context, nullptr);
    if (code != RKNN_SUCC) {
      std::cerr << "rknn_run failed " << code << '\n';
      rknn_destroy(context);
      return 6;
    }
    std::vector<rknn_output> outputs(io.n_output);
    for (uint32_t i = 0; i < io.n_output; ++i) {
      outputs[i].index = i;
      outputs[i].want_float = 1;
    }
    if (rknn_outputs_get(context, io.n_output, outputs.data(), nullptr) != RKNN_SUCC) {
      std::cerr << "outputs_get failed\n";
      rknn_destroy(context);
      return 7;
    }
    rknn_outputs_release(context, io.n_output, outputs.data());
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    if (iteration >= 2) times.push_back(elapsed);
  }
  const double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
  std::cout << "runs=" << times.size() << " mean_ms=" << mean
            << " inference_fps=" << 1000.0 / mean << '\n';
  rknn_destroy(context);
  return 0;
}
