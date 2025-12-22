cmake_minimum_required(VERSION 3.5)
project(psub1)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일러 경고 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # cmake에서 지원하지 않는 ament_target_dependencies(),
# ament_package() 명령의 정의를 포함하고 있음, ros2에서 추가된 기능
find_package(rclcpp REQUIRED) # ROS2 라이브러리
find_package(std_msgs REQUIRED) # interface 라이브러리

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

#build
add_executable(pub src/pub.cpp) # 실행파일 생성 / 소스파일 컴파일하여
ament_target_dependencies(pub rclcpp std_msgs) # 외부 라이브러리 가지고 오는 거

add_executable(sub src/sub.cpp) # 실행파일 생성 / 소스파일 컴파일하여
ament_target_dependencies(sub rclcpp std_msgs) # 외부 라이브러리 가지고 오는 거


# Install
install(TARGETS # 실행파일을 어디에 설치할지 
  pub # 설치 대상
  sub
  DESTINATION lib/${PROJECT_NAME}) # 설치할 위치

ament_package() # ament(ROS 2 빌드 시스템)를 쓰는 패키지라고 선언
