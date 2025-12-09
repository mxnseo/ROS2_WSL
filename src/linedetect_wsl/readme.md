LineDetector

ROS2 노드 기반의 라인 추적 시스템임.
ROS2와 OpenCV 사용하여 카메라 이미지 전처리, 라인 후보 레이블링, 타깃 라인 추적까지 수행함.

1. 개요

압축 이미지 토픽을 구독한 후 다음의 작업 수행함:

이미지 흑백 변환, 밝기 보정, 이진화

ROI(하단 영역)만 사용해 라인 후보 분리

connectedComponentsWithStats로 blob 추출

첫 프레임은 중앙과 가장 가까운 blob 선택

이후 프레임은 이전 타깃 위치와 가장 가까운 blob 추적

선택 결과 시각화

중앙 기준 오프셋(error) 계산

2. 소스 코드 구조
2.1 생성자

ROS2 노드 생성, 초기값 설정, CompressedImage 구독함.

LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_(320, 60), first_run_(true) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12",
        qos_profile,
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
}


기능 요약:

tmp_pt_ 초기값 (320, 60) 사용

first_run_ 플래그 true로 설정

"/image/compressed_12" 토픽 구독

3. 이미지 전처리
3.1 preprocess_image

흑백 변환 → 밝기 보정 → 이진화 → ROI 추출 순서로 진행함.

cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Scalar bright_avg = cv::mean(frame_gray);
    frame_gray = frame_gray + (100 - bright_avg[0]);

    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);

    return frame_binary(cv::Rect(0, 240, 640, 120));
}


설명:

목표 밝기값을 100으로 맞추는 간단한 normalization 수행함

threshold 130 기준으로 이진화

이미지 하단 240~360 영역만 ROI로 사용함

4. 라인 후보 탐색 알고리즘
4.1 find_target_line

connectedComponentsWithStats 결과(stats, centroids)를 기반으로 타깃 라인 선택함.

int LineDetector::find_target_line(const cv::Mat& roi, const cv::Mat& stats, const cv::Mat& centroids) {
    int cnt = stats.rows;

    if (first_run_) {
        int best_idx = -1;
        int min_center_dist = 10000;

        for (int i = 1; i < cnt; i++) {
            int area = stats.at<int>(i, 4);
            if (area > 100) {
                int cx = cvRound(centroids.at<double>(i, 0));
                int dist_from_center = abs(320 - cx);

                if (dist_from_center < min_center_dist) {
                    min_center_dist = dist_from_center;
                    best_idx = i;
                }
            }
        }

        if (best_idx != -1) {
            tmp_pt_ = cv::Point(cvRound(centroids.at<double>(best_idx, 0)), cvRound(centroids.at<double>(best_idx, 1)));
            first_run_ = false;
        }
    }

    int min_idx = -1;
    int min_dist = roi.cols;
    int search_radius = 60;

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int dist = cv::norm(cv::Point(x, y) - tmp_pt_);

            if (dist < min_dist && dist <= search_radius) {
                min_dist = dist;
                min_idx = i;
            }
        }
    }

    if (min_idx != -1) {
        tmp_pt_ = cv::Point(cvRound(centroids.at<double>(min_idx, 0)), cvRound(centroids.at<double>(min_idx, 1)));
    }

    return min_idx;
}


동작 방식:

첫 프레임

화면 중앙(320px) 기준으로 가장 가까운 라인 선택

area > 100 조건으로 노이즈 제거

선택된 centroid를 tmp_pt_에 저장

이후 프레임

tmp_pt_ 주변에서 search_radius=60 이내 blob 탐색

가장 가까운 blob을 타깃으로 선택

선택 실패 시 기존 타깃 유지됨

5. 시각화
5.1 draw_result

선택된 blob은 빨간색, 나머지는 파란색으로 표시함.

void LineDetector::draw_result(cv::Mat& result, const cv::Mat& stats, const cv::Mat& centroids, int target_idx) {
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    int cnt = stats.rows;
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int left = stats.at<int>(i, 0);
            int top = stats.at<int>(i, 1);
            int width = stats.at<int>(i, 2);
            int height = stats.at<int>(i, 3);

            cv::Scalar color = (i == target_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::rectangle(result, cv::Rect(left, top, width, height), color, 1);
            cv::circle(result, cv::Point(x, y), 5, color, -1);
        }
    }
}

6. 메인 콜백
6.1 mysub_callback

토픽 수신 → 전처리 → 라벨링 → 타깃 선택 → 시각화 순서로 동작함.

void LineDetector::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    auto startTime = chrono::steady_clock::now();
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;

    cv::Mat roi = preprocess_image(frame_color);

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    int target_idx = find_target_line(roi, stats, centroids);

    cv::Mat result_view = roi.clone();
    draw_result(result_view, stats, centroids, target_idx);

    int error = 320 - tmp_pt_.x;

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, time:%.2f ms", error, totalTime);

    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
}


설명:

압축 이미지 디코딩

ROI 전처리

connectedComponentsWithStats 적용

find_target_line으로 타깃 blob 선택

시각화

error 값 계산 (320 - centroid_x)

처리 시간 출력

7. 로그 출력 예시
Received Image : err:-12, time:13.45 ms
