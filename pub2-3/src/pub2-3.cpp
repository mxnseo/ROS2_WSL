#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <functional>
//#include <string>
using namespace std::chrono_literals;

// callback은 매개변수 없어야 함
// node, shardptr 매개변수로 선언해서 받도록 함
void callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mypub) {
    geometry_msgs::msg::Twist turtle_key; // 자료형을 갖는 객체 생성, class
    char key; // 문자 하나 받는 변수 선언
    // 멤버 함수 안에 잘 들어갔는지 확인, get logger
    std::cin >> key; // key값 받음
    
    switch (key) { // switch로 조건문 제어
    case 'f': // 전진
        turtle_key.linear.x = 1.0;
        turtle_key.angular.z = 0.0; // 초기화를 계속 해서 이전값 남기지 않게 함
        break;
    case 'b': // 후진
        turtle_key.linear.x = -1.0;
        turtle_key.angular.z = 0.0;
        break;
    case 'l': // 좌회전
        turtle_key.linear.x = 0.0;
        turtle_key.angular.z = 1.0;
        break;
    case 'r': // 우회전
        turtle_key.linear.x = 0.0;
        turtle_key.angular.z = -1.0;
        break;
    default:
        turtle_key.linear.x = 0.0;
        turtle_key.angular.z = 0.0;
        RCLCPP_INFO(node->get_logger(), "Publish: %s", "지원하는 키가 아님..."); // 로그 출력
        break;
    }

    RCLCPP_INFO(node->get_logger(), "linear: %f %f %f // angular: %f %f %f", 
    turtle_key.linear.x, turtle_key.linear.y, turtle_key.linear.z, turtle_key.angular.x, turtle_key.angular.y, turtle_key.angular.z); // 로그 출력
    mypub->publish(turtle_key); // 전송할 객체를 줌
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub2_3"); // 리턴값 shared pointer 객체
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 통신 설정

    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile); // /turtle1/cmd_vel 토픽을 발행

    // 리턴값은 void로 선언했는데 매개변수가 2개라 매개변수가 없어야 하는 상황
    // bind를 사용해서 새로운 함수 객체인 fn 반환 void() 형태
    // create_wall_timer는 void() 형태를 받기 때문임
    std::function<void()> fn = std::bind(callback, node, pub);
    auto timer = node->create_wall_timer(100ms, fn); // 주기 = 0.1초, 따라서 주파수(1초) = 10번
    
    rclcpp::spin(node); // 무한 대기 및 ctrl+c -> 종료
    rclcpp::shutdown();
    return 0;
}