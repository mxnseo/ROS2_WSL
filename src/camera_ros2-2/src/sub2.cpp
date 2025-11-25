#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

using std::placeholders::_1;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    static cv::VideoWriter video_writer; // static으로 선언하여 함수가 종료되어도 객체 유지
    static bool is_writer_initialized = false; // 초기화 여부 확인 플래그

    // 아직 초기화가 안 됐다면 (첫 번째 프레임이 들어왔을 때)
    if (!is_writer_initialized) {
        // 저장할 파일 이름, 코덱, FPS, 프레임 크기(가로, 세로), 컬러 여부
        video_writer.open("output_video.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(frame.cols, frame.rows), true);
        
        if (video_writer.isOpened()) {
            RCLCPP_INFO(node->get_logger(), "Video Writer Initialized!");
            is_writer_initialized = true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to open video writer");
        }
    }

    // 비디오 파일에 현재 프레임 쓰기
    if (is_writer_initialized) {
        video_writer.write(frame);
    }

    cv::imshow("wsl", frame);
    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl_12");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP
    
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed_12", qos_profile, fn);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}