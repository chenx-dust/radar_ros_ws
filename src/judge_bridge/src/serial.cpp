#include "judge_bridge/serial.hpp"
#include "judge_bridge/crc.hpp"
#include "judge_bridge/protocol.hpp"
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zstd.hpp>

using namespace JudgeBridge;

void JudgeSerial::SerialData_::clear(){
    header = {0,0,0,0};

    cmd_id = 0;
    tail = 0;
}


JudgeSerial::JudgePair JudgeSerial::get_front_frame(){
    if (frame_queue.empty()) {
        return {};
    }
    return frame_queue.front();
}

void JudgeSerial::write(CMD_ID command, uint8_t* data, uint16_t data_size){
    std::lock_guard<std::mutex> writing_lock(write_lock); 

    struct SerialData_ serial_data;
    serial_data.header.sof = 0xA5;
    serial_data.header.data_length = data_size;
    serial_data.header.package_sequence = seq;
    VerifyCRC::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&serial_data.header), sizeof(frame_header_t));
    serial_data.cmd_id = command;
    
    std::vector<uint8_t> full_pack(serial_data.header.data_length + sizeof(frame_header_t) + sizeof(uint16_t) * 2);
    auto header_arr = reinterpret_cast<std::array<uint8_t, sizeof(frame_header_t)>*>(&serial_data.header);
    auto cmd_id_arr = reinterpret_cast<std::array<uint8_t, sizeof(uint16_t)>*>(&serial_data.cmd_id);

    std::copy(header_arr->begin(), header_arr->end(), full_pack.begin());
    std::copy(cmd_id_arr->begin(), cmd_id_arr->end(), full_pack.begin() + sizeof(frame_header_t));
    std::copy(data, data + data_size, full_pack.begin() + sizeof(frame_header_t) + sizeof(uint16_t));

    VerifyCRC::Append_CRC16_Check_Sum(full_pack.data(), full_pack.size());

    boost::asio::write(serial,boost::asio::buffer(full_pack.data(), full_pack.size()));

    seq += 1;
}

JudgeSerial::JudgePair JudgeSerial::read(){
    //Reading data char by char, code is optimized for simplicity, not speed
    // std::lock_guard<std::mutex> read_lock(write_lock);

    using namespace boost;
    struct SerialData_ serial_data;

    do {
        asio::read(serial,asio::buffer(&serial_data.header.sof, 1));
    } while (serial_data.header.sof != 0xA5);

    asio::read(serial,asio::buffer(&serial_data.header.data_length, 2));
    asio::read(serial,asio::buffer(&serial_data.header.package_sequence, 1));
    asio::read(serial,asio::buffer(&serial_data.header.CRC8, 1));

    if(!VerifyCRC::Verify_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&serial_data.header), sizeof(frame_header_t))){
        throw std::runtime_error("CRC8 Check Failed");
    }

    std::vector<uint8_t> data_(serial_data.header.data_length);
    
    asio::read(serial,asio::buffer(&serial_data.cmd_id,2));
    asio::read(serial,asio::buffer(data_.data(),serial_data.header.data_length));
    asio::read(serial,asio::buffer(&serial_data.tail,2));

    recorder.write(reinterpret_cast<uint8_t*>(&serial_data.header), sizeof(frame_header_t));
    recorder.write(reinterpret_cast<uint8_t*>(&serial_data.cmd_id), 2);
    recorder.write(reinterpret_cast<uint8_t*>(data_.data()), serial_data.header.data_length);
    recorder.write(reinterpret_cast<uint8_t*>(&serial_data.tail), 2);

    std::vector<uint8_t> full_pack(serial_data.header.data_length + sizeof(frame_header_t) + sizeof(uint16_t) * 2);
    auto header_arr = reinterpret_cast<std::array<uint8_t, sizeof(frame_header_t)>*>(&serial_data.header);
    auto cmd_id_arr = reinterpret_cast<std::array<uint8_t, sizeof(uint16_t)>*>(&serial_data.cmd_id);
    auto tail_arr = reinterpret_cast<std::array<uint8_t, sizeof(uint16_t)>*>(&serial_data.tail);

    std::copy(header_arr->begin(), header_arr->end(), full_pack.begin());
    std::copy(cmd_id_arr->begin(), cmd_id_arr->end(), full_pack.begin() + sizeof(frame_header_t));
    std::copy(data_.begin(), data_.end(), full_pack.begin() + sizeof(frame_header_t) + sizeof(uint16_t));
    std::copy(tail_arr->begin(), tail_arr->end(), full_pack.begin() + sizeof(frame_header_t) + sizeof(uint16_t) + data_.size());

    // RCLCPP_INFO(rclcpp::get_logger("serial"), "%#x vs %#x", VerifyCRC::Get_CRC16_Check_Sum(data_.data(), serial_data.header.data_length, CRC_INIT), serial_data.tail);

    if (!VerifyCRC::Verify_CRC16_Check_Sum(full_pack.data(), full_pack.size())) {
        throw std::runtime_error("CRC16 Check Failed");
    }

    return {serial_data.cmd_id, data_};
}

void JudgeSerial::pop_frame(){
    if (!frame_queue.empty()) {
        frame_queue.pop();
    }
}


void JudgeSerial::Recorder::init(){
    if (is_enabled)
        return;
    boost::filesystem::path d_path = "./serial_recorder";
    if (!boost::filesystem::exists(d_path)) {
        RCLCPP_INFO(rclcpp::get_logger("RECORDER"), "Created directory: %s", d_path.c_str());
        boost::filesystem::create_directories(d_path);
    }
    std::string time_str = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());
    std::string filename_str = time_str + ".raw";
    boost::filesystem::path path = d_path / filename_str;

    file = boost::iostreams::file_sink(path.string(), std::ios::out | std::ios::binary);
    out.push(file.get());
    RCLCPP_INFO(rclcpp::get_logger("RECORDER"), "file opened: %s", path.c_str());
    is_enabled = true;
}

void JudgeSerial::Recorder::write(const uint8_t* buff, size_t len){
    if (!is_enabled)
        return;
    if (!file.is_initialized() && !file.get().is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("RECORDER"), "file is not open");
        return;
    }
    out.write(reinterpret_cast<const char*>(buff), len);
    out.flush();
}

void JudgeSerial::Recorder::stop(){
    if (!is_enabled) 
        return;
    is_enabled = false;
    out.flush();
    RCLCPP_INFO(rclcpp::get_logger("RECORDER"), "out.flush() finished");
    out.set_auto_close(true);
    out.reset();
    RCLCPP_INFO(rclcpp::get_logger("RECORDER"), "out.reset() finished, file closed: %d", !file.get().is_open());

}