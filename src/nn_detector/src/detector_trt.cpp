#ifdef TRT
#include <NvOnnxParser.h>
#include <cuda.h>
#include <detector/detector_trt.h>

#include <fstream>
#include <functional>
#include <ios>
#include <memory>
#include <sstream>

#ifndef CUDA_CHECK
#define CUDA_CHECK(callstr)                                                                   \
    {                                                                                         \
        cudaError_t error_code = callstr;                                                     \
        if (error_code != cudaSuccess) {                                                      \
            std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__ \
                      << std::endl;                                                           \
            exit(0);                                                                          \
        }                                                                                     \
    }
#endif

void DetectorTRT::TrtLogger::log(nvinfer1::ILogger::Severity severity, const char *msg) noexcept {
    switch (severity) {
        case Severity::kINTERNAL_ERROR:
        case Severity::kERROR:
            RCLCPP_ERROR(logger, msg);
            break;
        case Severity::kWARNING:
            RCLCPP_WARN(logger, msg);
            break;
        case Severity::kINFO:
            RCLCPP_INFO(logger, msg);
            break;
        case Severity::kVERBOSE:
            RCLCPP_DEBUG(logger, msg);
            break;
    }
}

DetectorTRT::DetectorTRT(const std::string &config_file, const std::string &share_dir,
                         const rclcpp::Logger &_logger, CUcontext *ctx)
    : NetDetector(config_file, share_dir, _logger), trt_logger(_logger), cuda_ctx(ctx) {
    std::string model_onnx = model_prefix + ".onnx";
    std::string model_cache = model_prefix + ".cache";

    RCLCPP_INFO(logger, "[TRT] loading cache: %s", model_cache.c_str());
    runtime = std::shared_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(trt_logger));

    std::stringstream egsr;
    egsr.seekg(0, std::ios::beg);
    std::ifstream egf(model_cache);
    if (!egf.is_open()) {
        build_engine(model_onnx, model_cache);
    } else {
        egsr << egf.rdbuf();

        egsr.seekg(0, std::ios::end);
        const int size = egsr.tellg();
        egsr.seekg(0, std::ios::beg);

        char *mem = (char *)malloc(size);
        egsr.read(mem, size);
        eg = std::shared_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(mem, size));
        if (eg == nullptr) build_engine(model_onnx, model_cache);
        free(mem);
        egf.close();
    }

    context = std::shared_ptr<nvinfer1::IExecutionContext>(eg->createExecutionContext());

    // input = new float[INPUT_H * INPUT_W * 3];
    input.resize(INPUT_H * INPUT_W * 3);

    int binding_num = eg->getNbIOTensors();
    // outputs = new float *[binding_num];
    outputs.resize(binding_num);
    out_size.resize(binding_num);
    layer_num = 0;

    for (int i = 0; i < binding_num; ++i) {
        const char *tname = eg->getIOTensorName(i);
        if (eg->getTensorIOMode(tname) == nvinfer1::TensorIOMode::kOUTPUT) {
            nvinfer1::Dims dims = eg->getTensorShape(tname);
            // assert(dims.nbDims == 5);
            // out_size[layer_num] = dims.d[0] * dims.d[1] * dims.d[2] * dims.d[3] * dims.d[4];
            out_size[layer_num] = 1;
            for (int j = 0; j < dims.nbDims; ++j) {
                out_size[layer_num] *= dims.d[j];
            }
            // outputs[layer_num] = new float[out_size[layer_num]];
            outputs[layer_num].resize(out_size[layer_num]);
            std::vector<size_t> standard_dims(dims.nbDims);
            std::transform(
                dims.d, dims.d + dims.nbDims, standard_dims.begin(),
                [](decltype(*dims.d) x) -> size_t {  // should be replaced by std::identity of C++20
                    return x;
                });

            decoder->set_layer_info(layer_num, standard_dims);
            ++layer_num;
        }
    }
    assert(layer_num + 1 == binding_num && "layer_num must equals binging_num-1");
    // create buffer array
    // buffers = new void *[layer_num + 1];
    buffers.resize(layer_num + 1);
    // input_layer
    cudaMalloc(&buffers[0], INPUT_H * INPUT_W * 3 * sizeof(float));
    // output_layer
    for (int i = 0; i < layer_num; ++i) {
        cudaMalloc(&buffers[i + 1], out_size[i] * sizeof(float));
    }

    for (int i = 0; i < binding_num; ++i) {
        context->setTensorAddress(eg->getIOTensorName(i), buffers[i]);
    }

    // cudaStreamCreate(&stream);

    RCLCPP_INFO(logger, "[TRT] engine loaded");
}

DetectorTRT::~DetectorTRT() {
    if (buffers.size() > 0 && buffers[0]) {
        cudaFree(buffers[0]);
        for (int i = 0; i < layer_num; ++i) {
            cudaFree(buffers[i + 1]);
        }
    }
}

void DetectorTRT::build_engine(const std::string &model_onnx, const std::string &model_cache) {
    RCLCPP_INFO(logger, "[TRT] fail to load engine; generating one from onnx file...");

    std::shared_ptr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(trt_logger)};
    std::shared_ptr<nvinfer1::IBuilderConfig> config{builder->createBuilderConfig()};

    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1 << 30);
    config->setFlag(nvinfer1::BuilderFlag::kFP16);

    auto flag =
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::shared_ptr<nvinfer1::INetworkDefinition> network{builder->createNetworkV2(flag)};
    std::shared_ptr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, trt_logger)};

    bool res = parser->parseFromFile(model_onnx.c_str(),
                                     static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));
    assert(res && "parse onnx file failed");

    std::shared_ptr<nvinfer1::IHostMemory> plan{builder->buildSerializedNetwork(*network, *config)};
    std::ofstream egf_o;
    egf_o.open(model_cache, std::ios::binary | std::ios::out);
    egf_o.write((const char *)plan->data(), plan->size());
    egf_o.close();
    eg = std::shared_ptr<nvinfer1::ICudaEngine>(
        runtime->deserializeCudaEngine(plan->data(), plan->size()));
}

std::vector<Armor> DetectorTRT::detect(cv::Mat img) {
    // PerfGuard perf_infer("Detector::Infer");

    float r = -1.;  // not resized
    if (img.cols != INPUT_W || img.rows != INPUT_H) {
        RCLCPP_WARN(logger, "[TRT INFER] resizing %dx%d to %dx%d", img.cols, img.rows, INPUT_W,
                    INPUT_H);
        r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
        img = static_resize(img, INPUT_H, INPUT_W);
    }

    assert(img.type() == CV_8UC3);
    assert(img.cols == INPUT_W && img.rows == INPUT_H);

    // RCLCPP_INFO(logger,"[TRT INFER] 1");

    img2blob(img, input);
    // 解决多线程infer问题
    cuCtxPushCurrent(*cuda_ctx);

    // cudaMemcpyAsync(buffers[0], input, INPUT_H * INPUT_W * 3 * sizeof(float),
    //                 cudaMemcpyHostToDevice, stream);

    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input.data(), INPUT_H * INPUT_W * 3 * sizeof(float),
                               cudaMemcpyHostToDevice, 0));

    // bool result = context->enqueueV2(buffers, stream, nullptr);

    bool result = context->executeV2(&buffers[0]);
    if (!result) {
        RCLCPP_ERROR(logger, "[TRT INFER] forward failed");
        // logger.error();
        return {};
    }

    for (int i = 0; i < layer_num; ++i) {
        // cudaMemcpyAsync(outputs[i], buffers[i + 1], out_size[i] * sizeof(float),
        //                 cudaMemcpyDeviceToHost, stream);
        CUDA_CHECK(cudaMemcpyAsync(outputs[i].data(), buffers[i + 1], out_size[i] * sizeof(float),
                                   cudaMemcpyDeviceToHost, 0));
    }

    // cudaStreamSynchronize(stream);

    // 解决多线程infer问题
    cuCtxPopCurrent(cuda_ctx);

    std::vector<Armor> res;
    for (int i = 0; i < layer_num; ++i) {
        decoder->decode(i, outputs[i].data(), res);
    }

    // remove items out of the bounds
    auto last = std::remove_if(
        res.begin(), res.end(), [n = point_num, H = INPUT_H, W = INPUT_W](auto item) {
            return std::any_of(item.pts, item.pts + n,
                               [=](auto p) { return p.x < 0 || p.x > W || p.y < 0 || p.y > H; });
        });
    res.erase(last, res.end());

    res = do_merge_nms(res);

    if (r > 0) {
        for (auto &a : res) {
            a.rect.x /= r;
            a.rect.y /= r;
            a.rect.width /= r;
            a.rect.height /= r;
            for (auto &p : a.pts) {
                p.x /= r;
                p.y /= r;
            }
        }
    }

    return res;
}

#endif