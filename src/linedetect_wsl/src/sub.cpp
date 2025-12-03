#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;
  
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{   
    // ROS 메시지의 압축된 데이터(msg->data)를 OpenCV의 Mat 객체로 복원(디코딩)함
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Scalar bright_avg = mean(frame_gray); // 밝기 평균
    frame_gray = frame_gray + (100 - bright_avg[0]);

    // 이진화 후 crop
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 150, 255, cv::THRESH_BINARY);
    
    cv::Mat roi = frame_binary(cv::Rect(0, 240, 640, 120));

    
    // 객체 찾는 부분
    static cv::Point tmp_pt(320, 60);
    cv::Mat labels, stats, centroids;
    int cnt = cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    int min_idx = -1;
    int min_dist = roi.cols;

    for (int i=1; i<cnt; i++){
        int area = stats.at<int>(i, 4);

        if(area > 100){
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int dist = cv::norm(cv::Point(x, y) - tmp_pt);

            if(dist < min_dist && dist <= 150){
                min_dist = dist;
                min_idx = i;
            }
        }
    }

    if(min_idx != -1 && min_dist <= 150){
        tmp_pt = cv::Point(cvRound(centroids.at<double>(min_idx, 0)), cvRound(centroids.at<double>(min_idx, 1)));
    }
    else cv::circle(roi, cv::Point(tmp_pt.x, tmp_pt.y), 5, cv::Scalar(0, 0, 255), -1);

    cv::Mat result;
    cv::cvtColor(roi, result, cv::COLOR_GRAY2BGR);

    for (int i = 1; i < stats.rows; i++) {
       int area = stats.at<int>(i, 4);
       if (area > 100) {
           int x = cvRound(centroids.at<double>(i, 0));
           int y = cvRound(centroids.at<double>(i, 1));
           if (x == tmp_pt.x) {
               cv::rectangle(result, cv::Rect(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3)), cv::Scalar(0, 0, 255));
               cv::circle(result, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
           }
           else {
               cv::rectangle(result, cv::Rect(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3)), cv::Scalar(255, 0, 0));
               cv::circle(result, cv::Point(x, y), 5, cv::Scalar(255, 0, 0), -1);
           }
           
       }
    }
    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result);

    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame_color.rows, frame_color.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl_12");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //TCP
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //UDP
    
    // 콜백 함수를 담을 수 있는 함수 객체(fn)를 선언함. 매개변수로 메시지 하나만 받는 형태임
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    
    // std::bind를 써서 mysub_callback 함수의 첫 번째 인자(node)를 미리 고정시킴
    // _1은 나중에 들어올 메시지(msg)가 들어갈 자리임
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed_12", qos_profile, fn);
    
    // 노드가 죽지 않고 계속 살아서 메시지를 기다리게 함 (무한 루프)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}