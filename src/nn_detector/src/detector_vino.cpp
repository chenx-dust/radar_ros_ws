#ifndef TRT
#define VINO_FALLBACK
#endif

#ifdef VINO_FALLBACK
#include <detector/detector_vino.h>

DetectorVINO::DetectorVINO(const std::string &config_file, const std::string &share_dir,
                           const rclcpp::Logger &_logger)
    : NetDetector(config_file, share_dir, _logger) {
    std::string model_xml = model_prefix + ".xml";
    std::string model_bin = model_prefix + ".bin";
    RCLCPP_INFO(logger, "[VINO] loading model: %s", model_bin.c_str());

    core.set_property(ov::cache_dir(share_dir + "/net_cache"));
    std::shared_ptr<ov::Model> model = core.read_model(model_xml, model_bin);
    compiled_model = core.compile_model(model, "GPU");

    layer_num = model->get_output_size();
    for (int i = 0; i < layer_num; ++i) {
        auto output = compiled_model.output(i);
        // ov::Shape is a derived class of std::vector<size_t>
        const std::vector<size_t> *pDims = &output.get_shape();
        // output.get_index() always returns 0, I don't know why.
        decoder->set_layer_info(i, *pDims);
    }

    infer_request = compiled_model.create_infer_request();

    RCLCPP_INFO(logger, "[VINO] model loaded");
}

std::vector<Armor> DetectorVINO::detect(cv::Mat img) {
    float r = -1.;  // not resized
    if (img.cols != INPUT_W || img.rows != INPUT_H) {
        RCLCPP_WARN(logger, "[VINO INFER] resizing %dx%d to %dx%d", img.cols, img.rows, INPUT_W,
                    INPUT_H);
        r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
        img = static_resize(img, INPUT_H, INPUT_W);
    }
    // RCLCPP_INFO(logger, "r  %f", r);
    assert(img.type() == CV_8UC3);
    assert(img.cols == INPUT_W && img.rows == INPUT_H);

    ov::Tensor input_tensor = infer_request.get_input_tensor();
    auto data = input_tensor.data<float>();
    img2blob(img, data);

    infer_request.infer();

    std::vector<Armor> res;

    for (int i = 0; i < layer_num; ++i) {
        ov::Tensor output_tensor = infer_request.get_output_tensor(i);
        const float *out_data = output_tensor.data<const float>();
        decoder->decode(i, out_data, res);
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