cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Keep ament/CMake Python helpers on the distro interpreter instead of user-local ones.
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python3 interpreter for ament helpers" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter for legacy CMake helpers" FORCE)
endif()

include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  sensor_msgs
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(GPIOD_LIBRARY gpiod)
find_package(Threads REQUIRED)

set(UNITREE_SDK2_ROOT "/home/guobing/unitree_sdk2")
set(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(UNITREE_SDK2_THIRDPARTY_DIR "${UNITREE_SDK2_ROOT}/thirdparty")
set(UNITREE_SDK2_THIRDPARTY_LIB_DIR "${UNITREE_SDK2_THIRDPARTY_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}")
set(INSPIRE_HAND_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../inspire_hand_ws")
set(INSPIRE_HAND_IDL_DIR "${INSPIRE_HAND_WS_ROOT}/inspire_hand_sdk/hand_idl")

find_library(UNITREE_SDK2_EXTERNAL_LIB unitree_sdk2 PATHS ${UNITREE_SDK2_LIB_DIR} NO_DEFAULT_PATH)
find_library(UNITREE_DDSC_EXTERNAL_LIB ddsc PATHS ${UNITREE_SDK2_THIRDPARTY_LIB_DIR} NO_DEFAULT_PATH)
find_library(UNITREE_DDSCXX_EXTERNAL_LIB ddscxx PATHS ${UNITREE_SDK2_THIRDPARTY_LIB_DIR} NO_DEFAULT_PATH)

if(UNITREE_SDK2_EXTERNAL_LIB AND UNITREE_DDSC_EXTERNAL_LIB AND UNITREE_DDSCXX_EXTERNAL_LIB AND EXISTS "${INSPIRE_HAND_IDL_DIR}/inspire_hand_state.hpp")
  add_library(ddsc_external SHARED IMPORTED GLOBAL)
  set_target_properties(ddsc_external PROPERTIES
    IMPORTED_LOCATION ${UNITREE_DDSC_EXTERNAL_LIB}
    IMPORTED_NO_SONAME TRUE)
  target_link_libraries(ddsc_external INTERFACE Threads::Threads)
  target_include_directories(ddsc_external INTERFACE ${UNITREE_SDK2_THIRDPARTY_DIR}/include)

  add_library(ddscxx_external SHARED IMPORTED GLOBAL)
  set_target_properties(ddscxx_external PROPERTIES
    IMPORTED_LOCATION ${UNITREE_DDSCXX_EXTERNAL_LIB}
    IMPORTED_NO_SONAME TRUE)
  target_link_libraries(ddscxx_external INTERFACE Threads::Threads ddsc_external)
  target_include_directories(ddscxx_external INTERFACE
    ${UNITREE_SDK2_THIRDPARTY_DIR}/include
    ${UNITREE_SDK2_THIRDPARTY_DIR}/include/ddscxx)

  add_library(unitree_sdk2_external STATIC IMPORTED GLOBAL)
  set_target_properties(unitree_sdk2_external PROPERTIES
    IMPORTED_LOCATION ${UNITREE_SDK2_EXTERNAL_LIB})
  target_link_libraries(unitree_sdk2_external INTERFACE ddsc_external ddscxx_external Threads::Threads)
  target_include_directories(unitree_sdk2_external INTERFACE
    ${UNITREE_SDK2_ROOT}/include
    ${UNITREE_SDK2_THIRDPARTY_DIR}/include
    ${UNITREE_SDK2_THIRDPARTY_DIR}/include/ddscxx)
endif()

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(low_level_ctrl_hg src/h1-2/lowlevel/low_level_ctrl_hg.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_low_state_hg src/read_low_state_hg.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
#add_executable(record_bag src/record_bag.cpp)
add_executable(go2_sport_client src/go2/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
target_compile_features(go2_sport_client PRIVATE cxx_std_20)

add_executable(go2_stand_example src/go2/go2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(go2_stand_example PRIVATE cxx_std_20)

add_executable(go2_robot_state_client src/go2/go2_robot_state_client.cpp src/common/motor_crc.cpp) 
target_compile_features(go2_robot_state_client PRIVATE cxx_std_20)

add_executable(g1_arm_sdk_dds_example src/g1/high_level/g1_arm_sdk_dds_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_arm_sdk_dds_example PRIVATE cxx_std_20)

add_executable(g1_lowstate_to_joint_states src/g1/high_level/g1_lowstate_to_joint_states.cpp)
target_compile_features(g1_lowstate_to_joint_states PRIVATE cxx_std_20)

if(TARGET unitree_sdk2_external)
  add_executable(
    g1_inspire_hand_to_joint_states
    src/g1/high_level/g1_inspire_hand_to_joint_states.cpp
    ${INSPIRE_HAND_IDL_DIR}/inspire_hand_state.cpp)
  target_compile_features(g1_inspire_hand_to_joint_states PRIVATE cxx_std_17)
  target_include_directories(g1_inspire_hand_to_joint_states PRIVATE ${INSPIRE_HAND_IDL_DIR})
  set_target_properties(g1_inspire_hand_to_joint_states PROPERTIES
    BUILD_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}")

  add_executable(
    g1_inspire_hand_touch_to_visualization
    src/g1/high_level/g1_inspire_hand_touch_to_visualization.cpp
    ${INSPIRE_HAND_IDL_DIR}/inspire_hand_touch.cpp)
  target_compile_features(g1_inspire_hand_touch_to_visualization PRIVATE cxx_std_17)
  target_include_directories(g1_inspire_hand_touch_to_visualization PRIVATE ${INSPIRE_HAND_IDL_DIR})
  set_target_properties(g1_inspire_hand_touch_to_visualization PROPERTIES
    BUILD_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}"
    INSTALL_RPATH "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}")
endif()

add_executable(g1_arm_action_example src/g1/high_level/g1_arm_action_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_arm_action_example PRIVATE cxx_std_20)

add_executable(g1_dex3_example src/g1/dex3/g1_dex3_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_dex3_example PRIVATE cxx_std_20)

add_executable(g1_loco_client_example src/g1/high_level/loco_client_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_loco_client_example PRIVATE cxx_std_20)

add_executable(g1_loco_service_example src/g1/high_level/g1_loco_service_example.cpp)
target_compile_features(g1_loco_service_example PRIVATE cxx_std_20)

add_executable(g1_motion_switcher_example src/g1/high_level/g1_motion_switcher_example.cpp)
target_compile_features(g1_motion_switcher_example PRIVATE cxx_std_20)

if(GPIOD_LIBRARY)
  add_executable(g1_emergency_stop_node src/g1/high_level/g1_emergency_stop_node.cpp)
  target_compile_features(g1_emergency_stop_node PRIVATE cxx_std_20)
else()
  message(WARNING "libgpiod not found: skipping g1_emergency_stop_node. Install libgpiod-dev to enable it.")
endif()

add_executable(g1_ankle_swing_example src/g1/lowlevel/g1_ankle_swing_example.cpp src/common/motor_crc_hg.cpp)
target_compile_features(g1_ankle_swing_example PRIVATE cxx_std_20)

add_executable(g1_audio_client_example src/g1/audio_client/g1_audio_client_example.cpp)
target_compile_features(g1_audio_client_example PRIVATE cxx_std_20)

add_executable(g1_tts_led_control_example src/g1/audio_client/g1_tts_led_control_example.cpp)
target_compile_features(g1_tts_led_control_example PRIVATE cxx_std_20)

add_executable(b2w_sport_client src/b2w/b2w_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2w_sport_client PRIVATE cxx_std_20)

add_executable(b2w_stand_example src/b2w/b2w_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2w_stand_example PRIVATE cxx_std_20)

add_executable(b2_stand_example src/b2/b2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2_stand_example PRIVATE cxx_std_20)

add_executable(b2_sport_client src/b2/b2_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2_sport_client PRIVATE cxx_std_20)


ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(low_level_ctrl_hg ${DEPENDENCY_LIST})
ament_target_dependencies(g1_low_level_example ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state_hg ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
#ament_target_dependencies(record_bag ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2_stand_example ${DEPENDENCY_LIST})


ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})
ament_target_dependencies(g1_arm_sdk_dds_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_lowstate_to_joint_states ${DEPENDENCY_LIST})
if(TARGET g1_inspire_hand_to_joint_states)
  ament_target_dependencies(g1_inspire_hand_to_joint_states ${DEPENDENCY_LIST})
  target_link_libraries(g1_inspire_hand_to_joint_states unitree_sdk2_external)
endif()
if(TARGET g1_inspire_hand_touch_to_visualization)
  ament_target_dependencies(g1_inspire_hand_touch_to_visualization ${DEPENDENCY_LIST})
  target_link_libraries(g1_inspire_hand_touch_to_visualization unitree_sdk2_external)
endif()
ament_target_dependencies(g1_arm_action_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_dex3_example ${DEPENDENCY_LIST})
target_include_directories(g1_dex3_example PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(g1_loco_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_loco_service_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_motion_switcher_example ${DEPENDENCY_LIST})
if(TARGET g1_emergency_stop_node)
  ament_target_dependencies(g1_emergency_stop_node ${DEPENDENCY_LIST})
  target_link_libraries(g1_emergency_stop_node ${GPIOD_LIBRARY})
endif()
ament_target_dependencies(g1_ankle_swing_example ${DEPENDENCY_LIST})

ament_target_dependencies(g1_audio_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_tts_led_control_example ${DEPENDENCY_LIST})

ament_target_dependencies(b2w_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(b2w_stand_example ${DEPENDENCY_LIST})

ament_target_dependencies(b2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(b2_stand_example ${DEPENDENCY_LIST})

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    if (${yaml-cpp_VERSION} VERSION_GREATER_EQUAL "0.6")
        message(STATUS "Found yaml-cpp version ${yaml-cpp_VERSION}")
        add_executable(g1_dual_arm_example src/g1/lowlevel/g1_dual_arm_example.cpp src/common/motor_crc_hg.cpp)
        target_compile_features(g1_dual_arm_example PRIVATE cxx_std_20)
        ament_target_dependencies(g1_dual_arm_example yaml-cpp ${DEPENDENCY_LIST})
    else()
        message(STATUS "yaml-cpp version ${yaml-cpp_VERSION} is too old, skipping build of g1_dual_arm_example.")
    endif()
endif()

install(TARGETS
        low_level_ctrl
        low_level_ctrl_hg
        g1_low_level_example
        read_low_state
        read_low_state_hg
        read_motion_state
        read_wireless_controller
        go2_sport_client
        go2_stand_example
        go2_robot_state_client
        g1_arm_sdk_dds_example
        g1_lowstate_to_joint_states
        g1_arm_action_example
        g1_dex3_example
        g1_loco_client_example
        g1_loco_service_example
        g1_motion_switcher_example
        g1_ankle_swing_example
        g1_audio_client_example
        g1_tts_led_control_example
        DESTINATION lib/${PROJECT_NAME})

if(TARGET g1_inspire_hand_to_joint_states)
  install(TARGETS g1_inspire_hand_to_joint_states DESTINATION lib/${PROJECT_NAME})
endif()

if(TARGET g1_inspire_hand_touch_to_visualization)
  install(TARGETS g1_inspire_hand_touch_to_visualization DESTINATION lib/${PROJECT_NAME})
endif()

if(TARGET g1_emergency_stop_node)
  install(TARGETS g1_emergency_stop_node DESTINATION lib/${PROJECT_NAME})
endif()

if(yaml-cpp_FOUND)
    if (${yaml-cpp_VERSION} VERSION_GREATER_EQUAL "0.6")
      install(TARGETS g1_dual_arm_example DESTINATION lib/${PROJECT_NAME})
    endif()
endif()



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
