### youtube

outline : https://youtu.be/rzmweRMjIdQ?si=mDOGJ1IYiccnWdsr

inline : https://youtu.be/DTijTpPN0IQ?si=v5_ZWiHPjea5bof7



-----

# ROS2 Line Detector (sub.cpp)

## 1\. 노드 초기화 (Constructor)

클래스 생성자 부분임. 노드가 실행될 때 가장 먼저 호출되어 초기 설정을 수행함.

```cpp
LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_(320, 60), first_run_(true) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // 토픽 구독 설정
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
}
```

  * **초기화 리스트 (`: ...`)**:
      * `Node("camsub_wsl_12")`: ROS2 네트워크에서 사용할 노드 이름을 설정함.
      * `tmp_pt_(320, 60)`: 라인 위치를 기억할 변수를 화면 중앙 상단으로 초기화함.
      * `first_run_(true)`: 프로그램이 막 켜졌음을 알리는 플래그를 `true`로 설정함.
  * **QoS 설정**: 네트워크 데이터가 밀릴 경우를 대비해 최근 10개의 데이터만 유지(`KeepLast(10)`)하도록 설정함.
  * **구독 생성 (`create_subscription`)**:
      * `image/compressed_12`라는 토픽에서 메시지가 오면 `mysub_callback` 함수를 실행하라고 등록함.
      * `std::bind`를 사용해 클래스 멤버 함수를 콜백으로 연결함.

## 2\. 영상 전처리 (Preprocessing)

원본 이미지를 분석하기 좋은 형태로 가공하는 단계

```cpp
cv::Mat LineDetector::preprocess_image(const cv::Mat& frame_color) {
    // 1. 흑백 변환
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);

    // 2. 밝기 보정 (핵심 로직)
    cv::Scalar bright_avg = cv::mean(frame_gray); 
    frame_gray = frame_gray + (100 - bright_avg[0]);

    // 3. 이진화
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);

    // 4. ROI 설정
    return frame_binary(cv::Rect(0, 240, 640, 120));
}
```

  * **밝기 보정**: `cv::mean`으로 현재 이미지의 평균 밝기를 구함. 목표 밝기인 `100`과의 차이만큼 전체 픽셀 값을 더하거나 빼줌. 
  * **ROI (Region of Interest)**: 불필요한 배경(하늘, 먼 풍경)은 자르고, 차량 바로 앞 도로 영역(`y: 240~360`)만 잘라내어 연산 속도를 높임.

## 3\. 라인 탐색 및 추적 (Tracking Algorithm)

매 프레임마다 라인을 새로 찾는 게 아니라, \*\*'이전 위치'를 기억해서 추적(Tracking)\*\*하는 방식을 사용함.

### 3.1. 초기 진입 (First Run)

```cpp
if (first_run_) {
    // ... (중략)
    // 화면 중앙(320)과 가장 가까운 객체를 찾음
    int dist_from_center = abs(320 - cx);
    if (dist_from_center < min_center_dist) {
         // ... 최적 인덱스 갱신
    }
    // 찾았으면 위치 기억하고 플래그 끔
    if (best_idx != -1) {
        tmp_pt_ = ...;
        first_run_ = false;
    }
}
```

  * 프로그램 시작 직후에는 정보가 없으므로, 화면 정중앙(`x=320`)에 가장 가까운 라인을 찾아 타겟으로 설정함.
  * 타겟을 찾으면 `first_run_`을 `false`로 바꾸고 다음 단계(추적)로 넘어감.

### 3.2. 주행 중 추적 (Tracking Loop)

```cpp
int search_radius = 60; // 탐색 반경 제한

for (int i = 1; i < cnt; i++) {
    // ...
    // 이전 프레임 위치(tmp_pt_)와 현재 객체 간 거리 계산
    int dist = cv::norm(cv::Point(x, y) - tmp_pt_);

    // 설정된 반경(60px) 내에서 가장 가까운 객체 선택
    if (dist < min_dist && dist <= search_radius) {
        min_dist = dist;
        min_idx = i;
    }
}
```

  * **거리 기반 추적**: `tmp_pt_`(이전 프레임의 라인 위치)를 기준으로 거리를 계산함.
  * **탐색 범위 제한**: 아무리 가까워도 `search_radius` (60픽셀)보다 멀리 있는 객체는 무시함.


## 4\. 시각화 (Visualization)


```cpp
void LineDetector::draw_result(...) {
    // ...
    cv::Scalar color = (i == target_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
    cv::rectangle(result, ..., color, 1);
    cv::circle(result, ..., 5, color, -1);
}
```

  * **타겟 라인**: 실제 주행에 사용되는 라인은 \*\*빨간색(Red)\*\*으로 표시함.
  * **후보군**: 인식은 됐으나 선택받지 못한 다른 객체들은 \*\*파란색(Blue)\*\*으로 표시함.
  * 이를 통해 알고리즘이 어떤 라인을 보고 있는지 눈으로 쉽게 확인할 수 있음.

## 5\. 메인 콜백 함수 (Callback Workflow)

```cpp
void LineDetector::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 1. 디코딩: 압축 이미지를 OpenCV Mat으로 변환
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if(frame_color.empty()) return;

    // 2. 전처리 및 객체 검출(레이블링)
    cv::Mat roi = preprocess_image(frame_color);
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);

    // 3. 타겟 선정 (알고리즘 수행)
    int target_idx = find_target_line(roi, stats, centroids);

    // 4. 시각화 처리
    draw_result(result_view, stats, centroids, target_idx);

    // 5. 조향 에러 계산 (Control Value)
    int error = 320 - tmp_pt_.x;

    // 6. 결과 출력
    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d ...", error ...);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
}
```

  * 모든 함수들을 순서대로 호출하여 데이터를 처리함.
  * 최종적으로 계산된 `error` 값(`320 - 현재x좌표`)은 중앙에서 얼마나 벗어났는지를 나타냄. 이 값이 양수면 왼쪽으로, 음수면 오른쪽
