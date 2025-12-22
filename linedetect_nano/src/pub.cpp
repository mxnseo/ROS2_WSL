#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>

// 비디오 경로 문자열로 미리 저장해놓은 거
//std::string src = "./5_lt_cw_100rpm_out.mp4"; 
std::string src = "./7_lt_ccw_100rpm_in.mp4"; 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("campub_12");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // TCP 통신
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // UDP 통신
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed_12", qos_profile ); // 메세지 인터페이스 이름
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    rclcpp::WallRate loop_rate(30.0); // 초당 n장 보내는 중 (주기)

    cv::VideoCapture cap(src); // 동영상 가져옴

    if (!cap.isOpened()) { // 에러 처리
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame;

    while(rclcpp::ok())
    {
        cap >> frame; // 장면 캡쳐 계속 보냄 (주기마다)

        // 에러 처리
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "frame empty"); break;}

        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg(); // 스마트 포인터
        mypub->publish(*msg); // 값을 전달
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}