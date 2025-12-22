#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>
#include <functional>
//#include <string>
using namespace std::chrono_literals;

// callback은 매개변수 없어야 함
// node, shardptr 매개변수로 선언해서 받도록 함
void callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mypub) {
    static int count;
    auto message = std_msgs::msg::Int32();
    message.data = count; // count 라는 변수 하나 잡아서 증가
    RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);
    mypub->publish(message); // 전송
    count++;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub2_1"); // 리턴값 shared pointer 객체
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 통신 설정

    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체
    auto pub = node->create_publisher<std_msgs::msg::Int32>("topic_pub2_1", qos_profile);

    // 리턴값은 void로 선언했는데 매개변수가 2개라 매개변수가 없어야 하는 상황
    // bind를 사용해서 새로운 함수 객체인 fn 반환 void() 형태
    // create_wall_timer는 void() 형태를 받기 때문임
    std::function<void()> fn = std::bind(callback, node, pub);
    auto timer = node->create_wall_timer(100ms, fn); // 주기 = 0.1초, 따라서 주파수(1초) = 10번
    
    rclcpp::spin(node); // 무한 대기 및 ctrl+c -> 종료
    rclcpp::shutdown();
    return 0;
}