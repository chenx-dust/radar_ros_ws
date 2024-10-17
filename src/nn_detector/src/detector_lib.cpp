#include <detector/detector_lib.h>
#include <utils/common.h>

using namespace nn_detector;

DetectorLib::DetectorLib(const DetectorParams& params) : params(params) {
    RCLCPP_INFO(params.logger, "Starting Robomaster Detector Lib!");

    RCLCPP_INFO(params.logger, "share_dir: %s", params.node_dir.c_str());
    RCLCPP_INFO(params.logger, "armor_config: %s", params.armor_config.c_str());
    RCLCPP_INFO(params.logger, "enable_imshow: %s", params.enable_imshow ? "true" : "false");
    RCLCPP_INFO(params.logger, "debug: %s", params.debug ? "true" : "false");

    auto armor_toml_config = toml::parse(params.node_dir / params.armor_config);
    auto type = armor_toml_config.at("type").as_string();

    // build detectors
#ifdef TRT
    try {
        cuInit(0);

        CUresult res = cuDeviceGet(&cuda_device, 0);
        RCLCPP_WARN(params.logger, "CUDA DEVICE: %p res: %d", (void*)&cuda_device, res);
        res = cuCtxCreate(&thread_ctx, 0, cuda_device);
        RCLCPP_WARN(params.logger, "CUDA CREATE CTX %p res: %d", (void*)&thread_ctx, res);

        detector_armor = std::make_shared<DetectorTRT>(params.armor_config, params.node_dir,
                                                       params.logger, &thread_ctx);
    } catch (const std::exception& e) {
#ifdef VINO_FALLBACK
        RCLCPP_ERROR(params.logger, "Failed to create TRT detector: %s", e.what());
        RCLCPP_WARN(params.logger, "Falling back to VINO");
        detector_armor = std::make_shared<DetectorVINO>(params.armor_config, params.node_dir,
                                                        params.logger);
#else
        RCLCPP_ERROR(params.logger, "Failed to create TRT detector: %s", e.what());
        rclcpp::shutdown();
#endif
    }
#else
    detector_armor = std::make_shared<DetectorVINO>(params.armor_config, params.node_dir,
                                                    params.logger);
#endif
}

radar_interface::srv::Detect_Response::SharedPtr DetectorLib::detect(const cv::Mat& img) {
    // auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    // cv::Mat img(req->image.height, req->image.width, encoding2mat_type(req->image.encoding),
    //             req->image.data.data());
    // For test intra comms
    // RCLCPP_INFO(params.logger,"PID: %d PTR: %p",getpid(),(void*)img.data);

    auto rep = std::make_shared<radar_interface::srv::Detect_Response>();

    if (!detector_armor) {
        RCLCPP_ERROR(params.logger, "Invalid Detector!");
        rclcpp::shutdown();
    }

    std::vector<Armor> res;
    std::shared_ptr<Detector> now_detector;
    res = detector_armor->detect(img);
    now_detector = detector_armor;
    // RCLCPP_INFO(params.logger,"res size: %d",res.size());

    if (params.debug || params.enable_imshow) {
        now_detector->draw(img, res);
    }

    if (params.enable_imshow) {
        cv::imshow("detect", img);
        cv::waitKey(1);
    }
    // 二次包装
    // std::move 实现data资源零拷贝
    for (size_t i = 0; i < res.size(); ++i) {
        rep->detected_armors.emplace_back(Armor2Msg(res[i]));
    }
    return rep;
}
