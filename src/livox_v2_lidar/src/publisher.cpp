#include "livox_v2_lidar/publisher.hpp"

using namespace livox_v2_lidar;

void LidarPublisher::recv_spin()
{
    std::array<unsigned char, pc_msg_size> recv_buf;
    boost::system::error_code error;
    std::size_t recv_length = 0;
    while (rclcpp::ok() && socket->is_open()) {
        static bool need_start = true;
        if (need_start) {
            std::array<uint8_t, 64> buf;
            protocal::push_kv_start hs;
            socket->send_to(
                boost::asio::buffer(buf,
                    protocal::write_frame_buffer(buf.data(), protocal::PUSH_KEY_VALUE,
                        reinterpret_cast<const uint8_t*>(&hs), sizeof(hs), buf.size())),
                boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));
            need_start = false;
        }

        socket->async_receive(boost::asio::buffer(recv_buf),
            [&](const boost::system::error_code& error_, std::size_t length_) {
                error = error_;
                recv_length = length_;
            });
        ctx.restart();
        ctx.run_for(std::chrono::milliseconds(get_parameter("timeout_ms").as_int()));
        if (!ctx.stopped()) {
            socket->cancel();
            ctx.run();
        }
        if (error && error != boost::asio::error::message_size) {
            RCLCPP_ERROR(get_logger(), "Receiver error: %s", error.message().c_str());
            need_start = true;
            continue;
        }

        auto header = reinterpret_cast<protocal::header*>(recv_buf.data());
        if (header->version == 0xaa)
            continue;   // 忽略控制协议
        if (header->data_type.value() == protocal::msg_type::MSG_PCD1 && header->length.value() == pc_msg_size) {
            if (!protocal::check_header_pcd1(recv_buf.data())) {
                RCLCPP_ERROR(get_logger(), "Wrong pcd1 header");
                continue;
            }
            auto data = reinterpret_cast<protocal::pcd1_span*>(recv_buf.data() + sizeof(protocal::header));
            proccess_pcd1(*header, *data);
        } else {
            RCLCPP_ERROR(get_logger(), "Wrong data_type: %d, length: %d", header->data_type.value(), header->length.value());
        }
    }
}

LidarPublisher::LidarPublisher(const rclcpp::NodeOptions& options) : rclcpp::Node("lidar_v2", options)
{
    declare_parameter("batch_dot_num", 9600);
    declare_parameter("lidar_line", 6);
    declare_parameter("dest_ip", "192.168.1.100");
    declare_parameter("udp_port", 57000);
    declare_parameter("timeout_ms", 1000);

    // frame_id 由名命名空间决定
    auto ns = std::string_view(get_namespace());
    auto ns_pos = ns.rfind('/');
    if (ns_pos != std::string_view::npos && ns_pos + 1 < ns.size()) {
        frame_id = ns.substr(ns.rfind('/') + 1);
        frame_id.append("_frame");
    } else {
        frame_id = "default_lidar_frame";
    }
    RCLCPP_INFO(get_logger(), "frame_id: %s", frame_id.c_str());
    batch_dot_num = get_parameter("batch_dot_num").as_int();
    line_num = get_parameter("lidar_line").as_int();
    dest_ip = boost::asio::ip::make_address_v4(get_parameter("dest_ip").as_string());

    RCLCPP_INFO(get_logger(), "Params: pc_batch_size: %lu, line_num: %d",
        batch_dot_num, line_num);

    // 初始化
    pc_pub = create_publisher<PointCloud2>("pc_raw", rclcpp::QoS(rclcpp::KeepLast(10)));
    pc2_init();

    local_port = get_parameter("udp_port").as_int();
    int timeout_ms = get_parameter("timeout_ms").as_int();
    RCLCPP_INFO(get_logger(), "Params: udp_port: %d, timeout_ms: %d", local_port, timeout_ms);

    socket.emplace(ctx, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port));
    recv_thread = std::thread(&LidarPublisher::recv_spin, this);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(livox_v2_lidar::LidarPublisher)
