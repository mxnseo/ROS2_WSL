#include "lidarsim/sub.hpp"

LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_l(125, 125), tmp_pt_r(375, 125), first_run_(true), mode(false) {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
        
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "topic_dxlpub",
        qos_profile);
}

cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 100, 255, cv::THRESH_BINARY_INV);

    return frame_binary(cv::Rect(0, 0, 500, 250));
}

std::pair<int, int> LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;
    int l_idx = -1;
    int r_idx = -1;
    
    if (first_run_) {
        int min_left_dist = 10000;
        int min_right_dist = 10000;

        for (int i = 1; i < cnt; i++) { 
            int area = stats.at<int>(i, 4);
            if (area > 30) { 
                int cx = cvRound(centroids.at<double>(i, 0));
                int dist_from_left = abs(125 - cx);
                int dist_from_right = abs(375 - cx);

                if (dist_from_left < min_left_dist) {
                    min_left_dist = dist_from_left;
                    l_idx = i;
                }

                if (dist_from_right < min_right_dist) {
                    min_right_dist = dist_from_right;
                    r_idx = i;
                }                
            }
        }
        
        if (l_idx != -1) {
            tmp_pt_l = cv::Point(cvRound(centroids.at<double>(l_idx, 0)), cvRound(centroids.at<double>(l_idx, 1)));
            first_run_ = false;
        }
        if (r_idx != -1) {
            tmp_pt_r = cv::Point(cvRound(centroids.at<double>(r_idx, 0)), cvRound(centroids.at<double>(r_idx, 1)));
            first_run_ = false;
        }

    }

    int min_dist_l = 10000;
    int min_dist_r = 10000;
    int search_radius = 100; 

    int idx_r = -1;
    int idx_l = -1;

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 30) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            cv::Point cur_pt(x, y);

            int dist_l = cv::norm(cur_pt - tmp_pt_l); 
            int dist_r = cv::norm(cur_pt - tmp_pt_r);

            if (dist_l < min_dist_l && dist_l <= search_radius) {
                min_dist_l = dist_l;
                idx_l = i;
            }

            if (dist_r < min_dist_r && dist_r <= search_radius) {
                min_dist_r = dist_r;
                idx_r = i;
            }            
        }
    }

    if (idx_l != -1) tmp_pt_l = cv::Point(cvRound(centroids.at<double>(idx_l, 0)), cvRound(centroids.at<double>(idx_l, 1)));
    if (idx_r != -1) tmp_pt_r = cv::Point(cvRound(centroids.at<double>(idx_r, 0)), cvRound(centroids.at<double>(idx_r, 1)));

    return std::make_pair(idx_l, idx_r);
}

void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int left_idx, int right_idx)
{
    if (result.channels() == 1) cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    int cnt = stats.rows;

    cv::Point robot_pos(250, 250); 

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 30) {
            if (i == left_idx || i == right_idx) {
                int x = cvRound(centroids.at<double>(i, 0));
                int y = cvRound(centroids.at<double>(i, 1));
                
                int left = stats.at<int>(i, 0);
                int top = stats.at<int>(i, 1);
                int width = stats.at<int>(i, 2);
                int height = stats.at<int>(i, 3);

                cv::Scalar color; 
                if (i == left_idx) color = cv::Scalar(0, 255, 0);
                else color = cv::Scalar(0, 0, 255);

                cv::rectangle(result, cv::Rect(left, top, width, height), color, 2);
                cv::circle(result, cv::Point(x, y), 3, color, -1);
            }
        }
    }

    if (left_idx != -1) {
        int left = stats.at<int>(left_idx, 0);
        int top = stats.at<int>(left_idx, 1);
        int width = stats.at<int>(left_idx, 2);
        int height = stats.at<int>(left_idx, 3);
        
        cv::Point l_box_bottom_right(left + width, top + height);
        
        cv::arrowedLine(result, robot_pos, l_box_bottom_right, cv::Scalar(0, 255, 0), 1);
    }

    if (right_idx != -1) {
        int left = stats.at<int>(right_idx, 0);
        int top = stats.at<int>(right_idx, 1);
        int height = stats.at<int>(right_idx, 3);
        
        cv::Point r_box_bottom_left(left, top + height);
        
        cv::arrowedLine(result, robot_pos, r_box_bottom_left, cv::Scalar(0, 0, 255), 1);
    }

    int target_x = (tmp_pt_l.x + tmp_pt_r.x) / 2;
    cv::Point target_pos(target_x, 100); 
    
    cv::arrowedLine(result, robot_pos, target_pos, cv::Scalar(255, 0, 0), 1);

    cv::circle(result, tmp_pt_l, 3, cv::Scalar(0, 255, 0), -1); 
    cv::circle(result, tmp_pt_r, 3, cv::Scalar(0, 0, 255), -1); 
}

// 키보드처리
int LineDetector::getch(void) 
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool LineDetector::kbhit(void) 
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

void LineDetector::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto startTime = chrono::steady_clock::now();
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;

    cv::Mat roi = preprocess_image(frame_color);

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    std::pair<int, int> targets = find_target_line(roi, stats, centroids);

    int left_idx = targets.first;
    int right_idx = targets.second;

    draw_result(frame_color, stats, centroids, left_idx, right_idx);

    int error = 250 - ((tmp_pt_l.x + tmp_pt_r.x) / 2);

    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false;
        else if(ch == 's') mode = true;
    }

    if (mode) {
        vel.x = 100 - k * error;
        vel.y = -(100 + k * error);
    } 
    else {
        vel.x = 0;
        vel.y = 0;
    }

    pub_->publish(vel);

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, leftvel: %.2f, rightvel: %.2f, time:%.2f ms", error, vel.x, vel.y, totalTime);

    cv::imshow("frame_color", frame_color);
    cv::waitKey(1);
}