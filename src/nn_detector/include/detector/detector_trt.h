#ifndef CRH_2023_DETECTOR_TRT_HPP_
#define CRH_2023_DETECTOR_TRT_HPP_

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <detector/detector.h>
#include <cuda.h>

class DetectorTRT : public NetDetector {
   private:
    std::shared_ptr<nvinfer1::IRuntime> runtime;
    std::shared_ptr<nvinfer1::ICudaEngine> eg;
    std::shared_ptr<nvinfer1::IExecutionContext> context;
    // cudaStream_t stream;

    class TrtLogger : public nvinfer1::ILogger {
       private:
        rclcpp::Logger logger;

       public:
        explicit TrtLogger(const rclcpp::Logger &logger_) : logger(logger_) {}
        void log(nvinfer1::ILogger::Severity severity, const char *msg) noexcept override;
    };
    TrtLogger trt_logger;
    std::vector<float> input;
    std::vector<std::vector<float>> outputs;
    std::vector<void *> buffers;

    //解决回调函数不在一个线程的问题
    CUcontext* cuda_ctx;
    std::vector<int> out_size;
    void build_engine(const std::string &model_onnx, const std::string &model_engine);

   public:
    explicit DetectorTRT(const std::string &config_file, const std::string &share_dir,
                         const rclcpp::Logger &_logger, CUcontext *ctx);
    ~DetectorTRT();
    std::vector<Armor> detect(cv::Mat) override;
};

#endif