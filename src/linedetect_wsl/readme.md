# LineDetector
ROS2 노드 기반의 라인 추적 시스템임.  
카메라에서 압축 이미지 수신 후 전처리, 레이블링, 라인 선택, 시각화, 에러 계산까지 수행함.

---

## 1. 개요
압축 이미지 토픽을 구독한 뒤 다음 작업 수행함:

- 이미지 흑백 변환, 밝기 보정, 이진화
- ROI(하단 영역)만 사용해 라인 후보 분리
- connectedComponentsWithStats로 blob 추출
- 첫 프레임은 화면 중앙과 가장 가까운 blob 선택
- 이후 프레임은 이전 타깃과 가장 가까운 blob 추적
- 결과 bounding box 시각화
- 중앙 기준 오프셋(error) 계산

---

## 2. 코드 구조

### 2.1 생성자
노드를 생성하고 구독 설정함.

```cpp
LineDetector::LineDetector() : Node("camsub_wsl_12"), tmp_pt_(320, 60), first_run_(true) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed_12",
        qos_profile,
        bind(&LineDetector::mysub_callback, this, placeholders::_1));
}
