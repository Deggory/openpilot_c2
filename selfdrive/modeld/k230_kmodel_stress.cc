#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/util.h>

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;

namespace {

template <typename Shape>
size_t shape_count(const Shape &shape) {
  return std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<size_t>());
}

template <typename Shape>
void print_shape(const char *kind, size_t index, datatype_t datatype, const Shape &shape) {
  std::printf("%s[%zu] dtype=%d shape=[", kind, index, static_cast<int>(datatype->typecode()));
  for (size_t i = 0; i < shape.size(); ++i) {
    std::printf("%s%zu", i == 0 ? "" : ",", static_cast<size_t>(shape[i]));
  }
  std::printf("] count=%zu\n", shape_count(shape));
}

void require_float32(datatype_t datatype, const char *name) {
  if (datatype->typecode() != dt_float32) {
    throw std::runtime_error(std::string("K230 kmodel tensor is not float32: ") + name);
  }
}

double now_sec() {
  timespec ts = {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

}  // namespace

int main(int argc, char **argv) {
  const char *model_path = argc > 1 ? argv[1] : "models/supercombo.kmodel";
  const int iterations = argc > 2 ? std::atoi(argv[2]) : 300;

  std::ifstream model(model_path, std::ios::binary);
  if (!model) {
    std::fprintf(stderr, "failed to open %s\n", model_path);
    return 2;
  }

  interpreter interp;
  interp.load_model(model).expect("invalid K230 kmodel");

  std::vector<runtime_tensor> inputs;
  std::vector<size_t> input_bytes;
  inputs.reserve(interp.inputs_size());
  input_bytes.reserve(interp.inputs_size());
  for (size_t i = 0; i < interp.inputs_size(); ++i) {
    const auto desc = interp.input_desc(i);
    const auto shape = interp.input_shape(i);
    print_shape("input", i, desc.datatype, shape);
    require_float32(desc.datatype, "input");
    auto tensor = host_runtime_tensor::create(desc.datatype, shape, hrt::pool_shared).expect("cannot create input tensor");
    interp.input_tensor(i, tensor).expect("cannot set input tensor");

    auto mapped = tensor.impl()->to_host().unwrap()->buffer().as_host().unwrap()
                    .map(map_access_::map_write).unwrap().buffer();
    const size_t bytes = shape_count(shape) * sizeof(float);
    if (mapped.size() < bytes) {
      throw std::runtime_error("input tensor buffer too small");
    }
    std::memset(reinterpret_cast<char *>(mapped.data()), 0, bytes);
    hrt::sync(tensor, sync_op_t::sync_write_back, true).expect("input sync failed");
    inputs.push_back(tensor);
    input_bytes.push_back(bytes);
  }

  for (size_t i = 0; i < interp.outputs_size(); ++i) {
    const auto desc = interp.output_desc(i);
    const auto shape = interp.output_shape(i);
    print_shape("output", i, desc.datatype, shape);
    require_float32(desc.datatype, "output");
    auto tensor = host_runtime_tensor::create(desc.datatype, shape, hrt::pool_shared).expect("cannot create output tensor");
    interp.output_tensor(i, tensor).expect("cannot set output tensor");
  }

  std::printf("k230_kmodel_stress inputs=%zu outputs=%zu iterations=%d\n",
              interp.inputs_size(), interp.outputs_size(), iterations);
  std::fflush(stdout);

  double total_ms = 0.0;
  volatile float checksum = 0.0f;
  for (int i = 0; i < iterations; ++i) {
    for (size_t j = 0; j < inputs.size(); ++j) {
      auto mapped = inputs[j].impl()->to_host().unwrap()->buffer().as_host().unwrap()
                      .map(map_access_::map_write).unwrap().buffer();
      if (mapped.size() < input_bytes[j]) {
        throw std::runtime_error("input tensor buffer too small");
      }
      std::memset(reinterpret_cast<char *>(mapped.data()), 0, input_bytes[j]);
      hrt::sync(inputs[j], sync_op_t::sync_write_back, true).expect("input sync failed");
    }

    const double t0 = now_sec();
    interp.run().expect("K230 kmodel run failed");
    const double t1 = now_sec();
    total_ms += (t1 - t0) * 1000.0;

    for (size_t j = 0; j < interp.outputs_size(); ++j) {
      auto out = interp.output_tensor(j).expect("cannot get output tensor");
      auto mapped = out.impl()->to_host().unwrap()->buffer().as_host().unwrap()
                      .map(map_access_::map_read).unwrap().buffer();
      if (mapped.size() >= sizeof(float)) {
        checksum += *reinterpret_cast<const float *>(mapped.data());
      }
    }

    if ((i + 1) % 10 == 0 || i + 1 == iterations) {
      std::printf("iter=%d avg_ms=%.3f last_ms=%.3f checksum=%f\n",
                  i + 1, total_ms / static_cast<double>(i + 1),
                  (t1 - t0) * 1000.0, static_cast<double>(checksum));
      std::fflush(stdout);
    }
  }

  return 0;
}
