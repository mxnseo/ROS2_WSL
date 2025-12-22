#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp" // Vector3 쓰기 위해 헤더 파일 변경
#include <memory>
#include <chrono>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // 생성자
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2"); // 리턴값 shared pointer 객체
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 통신 설정

    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile);

    geometry_msgs::msg::Vector3 number; // 자료형을 갖는 객체 생성, class

    rclcpp::WallRate loop_rate(1.0); // 반복주파수를 저장하는 객체(단위 Hz)
    std::cin >> number.x >> number.y >> number.z; // 3개의 실수값을 키보드로부터 받음

    while(rclcpp::ok()) { // 반복문, ctrl+c 안 누르면 계속 반복
        // 멤버 함수 안에 잘 들어갔는지 확인, get logger
        RCLCPP_INFO(node->get_logger(), "Publish: %f %f %f", number.x, number.y, number.z); // 로그 출력
        mypub->publish(number); // 전송할 객체를 줌 

        //rclcpp::spin_some(node);
        loop_rate.sleep(); // 반복주파수에서 남은 시간 만큼 sleep, 1초 주기를 정확히 줌
    }

    rclcpp::shutdown();
    return 0;
}
