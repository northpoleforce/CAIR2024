# 设置cmake最低版本
cmake_minimum_required(VERSION 3.10)


# 设置项目名称
project(CAIR2024)
# 打印项目名称
message("Project Name: ${PROJECT_NAME}")


# 检查系统架构
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    # 如果系统架构是x86_64，设置ARCH变量为amd64
    set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    # 如果系统架构是aarch64，设置ARCH变量为arm64
    set(ARCH arm64)
endif()


# 指定OpenCV的路径
set(OpenCV_DIR "/home/jetson/CAIR2024/opencv/opencv-4.1.1/build")
# 查找OpenCV库
find_package(OpenCV 4.1.1 REQUIRED)
if(NOT OpenCV_FOUND)
    # 如果没有找到OpenCV，打印错误信息并退出
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()
# 打印OpenCV的版本
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# # 打印OpenCV的头文件和库文件的路径
# message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV library dir: ${OpenCV_LIB_DIR}")

# 查找X11库
find_package(X11 REQUIRED)
if (NOT X11_FOUND)
    message(FATAL_ERROR "X11 Library Not Found")
endif()
# 查找OpenGL库
find_package(GLUT REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 找到Python库
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# # 找到Python解释器
# find_package(PythonInterp 3 REQUIRED)
# find_package(PythonLibs 3 REQUIRED)


# 添加头文件目录，这样在编译时可以找到这些头文件
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/unitree_legged_sdk)
include_directories(${OpenCV_INCLUDE_DIRS})
# 添加链接库目录，这样在链接时可以找到这些库
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/unitree_legged_sdk/${ARCH})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/UnitreecameraSDK-main/${ARCH})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
# link_directories(/usr/local/lib)


# 设置编译标志：17表示C++17
set(CMAKE_CXX_STANDARD 17)
# 设置编译标志：-O3表示优化等级为3，-fPIC表示生成位置无关代码
set(CMAKE_CXX_FLAGS "-O3 -no-pie")
# 设置编译标志：-pthread表示使用多线程
set(EXTRA_LIBS -pthread libunitree_camera.a libunitree_legged_sdk.a tstc_V4L2_xu_camera udev systemlog 
    ${OpenCV_LIBS} 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${X11_LIBRARIES}
    ${PYTHON_LIBRARIES})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
# # 测试ArUco库
# set (TEST_ARUCO_FILES 
#     src/Camera.cpp
#     src/MaterialIdentify.cpp)
# add_executable(test_aruco ${TEST_ARUCO_FILES})
# target_link_libraries(test_aruco PRIVATE ${EXTRA_LIBS})

# # 测试相机
# set (TEST_VIDEO_FILES 
#     src/Camera.cpp
#     src/TestVideo.cpp)
# add_executable(test_video ${TEST_VIDEO_FILES})
# target_link_libraries(test_video PRIVATE ${EXTRA_LIBS})

# # 测试视觉道路导航-CPU
# set (TEST_VISION_CPU_FILES 
#     src/Camera.cpp
#     src/visionCPU.cpp)
# add_executable(test_vision_cpu ${TEST_VISION_CPU_FILES})
# target_link_libraries(test_vision_cpu PRIVATE ${EXTRA_LIBS})
# # 测试视觉道路导航-GPU
# set (TEST_VISION_GPU_FILES 
#     src/Camera.cpp
#     src/visionGPU.cpp)
# add_executable(test_vision_gpu ${TEST_VISION_GPU_FILES})
# target_link_libraries(test_vision_gpu PRIVATE ${EXTRA_LIBS})

# # 测试arucoTag
# set (TEST_ARUCO 
#     src/Camera.cpp
#     src/testAruco.cpp)
# add_executable(test_aruco ${TEST_ARUCO})
# target_link_libraries(test_aruco PRIVATE ${EXTRA_LIBS})

# # 测试tasksDetector
# set (TEST_TASKSDETECTOR 
#     src/Camera.cpp
#     src/materialDetector.cpp)
# add_executable(test_tasksDetector ${TEST_TASKSDETECTOR})
# target_link_libraries(test_tasksDetector PRIVATE ${EXTRA_LIBS})

# # 测试tagMapLocalization
# set (TEST_TAGMAPLOCALIZATION 
#     src/Camera.cpp
#     src/move.cpp
#     src/testTagMapLocalization.cpp)
# add_executable(test_tagMapLocalization ${TEST_TAGMAPLOCALIZATION})
# target_link_libraries(test_tagMapLocalization PRIVATE ${EXTRA_LIBS})

# 测试 testArm
set (TEST_ARM
    src/Info.cpp 
    src/Arm.cpp
    src/testArm.cpp)
add_executable(testArm ${TEST_ARM})
target_link_libraries(testArm PRIVATE ${EXTRA_LIBS})

# 测试 testLoad
set (TEST_LOAD
    src/Info.cpp 
    src/Arm.cpp
    src/testLoad.cpp)
add_executable(testLoad ${TEST_LOAD})
target_link_libraries(testLoad PRIVATE ${EXTRA_LIBS})

# 测试 testExtraCamera
set (TEST_EXTRACAMERA
    src/ExtraCamera.cpp
    src/visionCPU.cpp
    src/taskDetector.cpp
    src/ReadParams.cpp
    src/testExtraCamera.cpp)
add_executable(testExtraCamera ${TEST_EXTRACAMERA})
target_link_libraries(testExtraCamera PRIVATE ${EXTRA_LIBS})
# 测试 testFrontCamera
set (TEST_FRONTCAMERA
    src/Camera.cpp
    src/ReadParams.cpp
    src/taskDetector.cpp
    src/testFrontCamera.cpp)
add_executable(testFrontCamera ${TEST_FRONTCAMERA})
target_link_libraries(testFrontCamera PRIVATE ${EXTRA_LIBS})

# 测试 testMove
set (TEST_MOVE 
    src/Info.cpp
    src/Arm.cpp
    src/tasks.cpp
    src/materialDetector.cpp
    src/move.cpp
    src/Camera.cpp
    src/ExtraCamera.cpp
    src/visionCPU.cpp
    src/taskDetector.cpp
    src/ReadParams.cpp
    src/testMove.cpp)
add_executable(testMove ${TEST_MOVE})
target_link_libraries(testMove PRIVATE ${EXTRA_LIBS})

set (TEST_CONFIG 
    src/ReadParams.cpp
    src/testReadParams.cpp)
add_executable(testReadParams ${TEST_CONFIG})
target_link_libraries(testReadParams PRIVATE ${EXTRA_LIBS})


# 主程序
set (MAIN_FILES 
    src/Info.cpp
    src/Arm.cpp
    src/tasks.cpp
    src/materialDetector.cpp
    src/move.cpp
    src/Camera.cpp
    src/ExtraCamera.cpp
    src/visionCPU.cpp
    src/taskDetector.cpp
    src/ReadParams.cpp
    src/main.cpp)
add_executable(main ${MAIN_FILES})
target_link_libraries(main PRIVATE ${EXTRA_LIBS})