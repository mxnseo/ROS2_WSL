#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <functional>

void mysub_callback(rclcpp::Node::SharedPtr node, const std_msgs::msg::String::SharedPtr msg) {
    // shared point 객체라 -> 사용
    RCLCPP_INFO(node->get_logger(), "Received message: %s", msg->data.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sub1_1_node");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 리턴값은 매개변수 1개로 선언했는데 매개변수가 2개
    // bind를 사용해서 새로운 함수 객체인 fn 반환 void(const std_msgs::msg::String::SharedPtr) 형태로 1개인데 내부적으로는 2개 처리
    std::function<void(const std_msgs::msg::String::SharedPtr)> fn = std::bind(mysub_callback, node, std::placeholders::_1);
    // 섭스크라이버 객체 생성 (노드가 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체 
    auto mysub = node->create_subscription<std_msgs::msg::String>("pub1_1_topic", qos_profile,fn);
    // 발행 받을 토픽 이름 (pub1-1 토픽 이름과 같아야 함)
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}