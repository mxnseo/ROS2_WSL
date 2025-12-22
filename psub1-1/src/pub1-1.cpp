#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <chrono>
#include <functional>
//#include <string>
using namespace std::chrono_literals;

// node, shardptr 매개변수로 선언해서 받도록 함
void callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mypub) {
    static int count;
    auto message = std_msgs::msg::String();
    message.data = "Hello world! " + std::to_string(count++); // count 라는 변수 하나 잡아서 증가
    RCLCPP_INFO(node->get_logger(), "Publish: %s", message.data.c_str()); // string 변환
    mypub->publish(message); // 전송
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pub1_1_node");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체    
    auto pub = node->create_publisher<std_msgs::msg::String>("pub1_1_topic", qos_profile);

    // 리턴값은 void로 선언했는데 매개변수가 2개라 매개변수가 없어야 하는 상황
    // bind를 사용해서 새로운 함수 객체인 fn 반환 void() 형태
    // create_wall_timer는 void() 형태를 받기 때문임
    std::function<void()> fn = std::bind(callback, node, pub);
    auto timer = node->create_wall_timer(100ms, fn); // 콜백 함수를 호출할 시간 주기, 사용자 정의 콜백 함수
    // shared ptr 객체 리턴
    // 100 -> 0.1초 주기
    // 주파수는 1초 단위 -> 10번
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}