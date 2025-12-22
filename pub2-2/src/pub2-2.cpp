#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <chrono>
#include <functional>
//#include <string>
using namespace std::chrono_literals;

// callback은 매개변수 없어야 함
// node, shardptr 매개변수로 선언해서 받도록 함
void callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub) {
    geometry_msgs::msg::Vector3 number; // 자료형을 갖는 객체 생성, class
    std::cin >> number.x >> number.y >> number.z; // 3개의 실수값을 키보드로부터 받음
    RCLCPP_INFO(node->get_logger(), "Publish: %f %f %f", number.x, number.y, number.z); // 로그 출력
    mypub->publish(number); // 전송할 객체를 줌 
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub2_2"); // 리턴값 shared pointer 객체
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 통신 설정

    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체
    auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub2_2", qos_profile);

    // 리턴값은 void로 선언했는데 매개변수가 2개라 매개변수가 없어야 하는 상황
    // bind를 사용해서 새로운 함수 객체인 fn 반환 void() 형태
    // create_wall_timer는 void() 형태를 받기 때문임
    std::function<void()> fn = std::bind(callback, node, pub);
    auto timer = node->create_wall_timer(100ms, fn); // 주기 = 0.1초, 따라서 주파수(1초) = 10번
    
    rclcpp::spin(node); // 무한 대기 및 ctrl+c -> 종료
    rclcpp::shutdown();
    return 0;
}