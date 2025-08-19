cmake_minimum_required(VERSION 3.8)
project(rtecm_motor)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rclc REQUIRED)
find_package(rclc_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclc_parameter REQUIRED)
find_package(Threads REQUIRED)
find_package(pd_interfaces REQUIRED)

find_library(soem NAMES soem PATHS ${CMAKE_CURRENT_LIST_DIR}/lib)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/include/
  ${CMAKE_CURRENT_LIST_DIR}/include/osal
  ${CMAKE_CURRENT_LIST_DIR}/include/osal/linux
  ${CMAKE_CURRENT_LIST_DIR}/include/oshw
  ${CMAKE_CURRENT_LIST_DIR}/include/oshw/linux
  ${CMAKE_CURRENT_LIST_DIR}/include/soem
  # ${CMAKE_CURRENT_LIST_DIR}/../../interface
)


execute_process(COMMAND xeno-config --posix --ldflags
OUTPUT_VARIABLE XENO_LDFLAGS
OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")

 


add_executable(youda_pd src/youda_pd.c src/chry_ringbuffer.c)
target_include_directories(youda_pd PUBLIC include)
 
target_link_libraries(youda_pd   m pthread  ${soem})
ament_target_dependencies(youda_pd rcl rclc std_msgs pd_interfaces)

install(TARGETS
youda_pd
  DESTINATION lib/${PROJECT_NAME}
)



install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
