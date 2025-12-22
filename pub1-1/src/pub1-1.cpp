#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp" // 정수 쓰기 위해 헤더 파일 변경
#include <memory>
#include <chrono>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // 생성자
    auto node = std::make_shared<rclcpp::Node>("node_pub1_1"); // 리턴값 shared pointer 객체
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 통신 설정

    // 퍼블리셔 객체 생성 (노드가 퍼블리셔 기능을 수행하기 위함), 동적할당, 리턴값 shared pointer 객체
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1_1", qos_profile);

    std_msgs::msg::Int32 number; // 자료형을 갖는 객체 생성, class
    number.data = 0; // 보내고 싶은 변수값 설정, 멤버 변수, 초기값 0

    rclcpp::WallRate loop_rate(1.0); // 반복주파수를 저장하는 객체(단위 Hz)

    while(rclcpp::ok()) { // 반복문, ctrl+c 안 누르면 계속 반복
        // 멤버 함수 안에 잘 들어갔는지 확인, get logger
        RCLCPP_INFO(node->get_logger(), "Publish: %d", number.data);
        mypub->publish(number); // 전송할 객체를 줌 
        //rclcpp::spin_some(node);

        number.data += 1; // 1씩 증가해서 보냄
        loop_rate.sleep(); // 반복주파수에서 남은 시간 만큼 sleep, 1초 주기를 정확히 줌
    }

    rclcpp::shutdown();
    return 0;
}
