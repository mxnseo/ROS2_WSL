#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // Twist 쓰기 위해 헤더 파일 변경
#include <memory>
#include <chrono>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // 생성자
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2"); // 리턴값 shared pointer 객체
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 통신 설정

    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile); // /turtle1/cmd_vel 토픽을 발행

    geometry_msgs::msg::Twist turtle_key; // 자료형을 갖는 객체 생성, class
    rclcpp::WallRate loop_rate(1.0); // 반복주파수를 저장하는 객체(단위 Hz)
    
    char key; // 문자 하나 받는 변수 선언

    while(rclcpp::ok()) { // 반복문, ctrl+c 안 누르면 계속 반복
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

        //rclcpp::spin_some(node);
        loop_rate.sleep(); // 반복주파수에서 남은 시간 만큼 sleep, 1초 주기를 정확히 줌
    }

    rclcpp::shutdown();
    return 0;
}
