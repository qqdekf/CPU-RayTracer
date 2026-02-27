cmake_minimum_required (VERSION 2.8)
project(tinyraytracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 使用 file(GLOB) 收集源文件
file(GLOB SOURCES 
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    # ${PROJECT_SOURCE_DIR}/include/*.cpp
    # ${PROJECT_SOURCE_DIR}/include/*.hpp
       # 注意：通常 .cpp 不放 include 目录！
)

# 收集头文件（用于 IDE 显示，非必需）
file(GLOB HEADERS 
    ${PROJECT_SOURCE_DIR}/include/*.h
)

# 添加头文件搜索路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
# 可执行文件输出地址
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)