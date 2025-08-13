cmake_minimum_required(VERSION 3.0.2)
project(azd_ked_driver)

# C++11 以上
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

# ===== SOEM (system) =====
find_path(SOEM_INCLUDE_DIR
  NAMES ethercat.h ethercattype.h
  PATHS /usr/local/include /usr/include
  PATH_SUFFIXES soem
)
find_library(SOEM_LIB
  NAMES soem
  PATHS /usr/local/lib /usr/local/lib64 /usr/lib /usr/lib64
)

if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIB)
  message(WARNING
    "SOEM not found automatically.\n"
    "  - Set SOEM_INCLUDE_DIR to the folder containing ethercat.h (e.g., /usr/local/include/soem)\n"
    "  - Set SOEM_LIB to the libsoem library file (e.g., /usr/local/lib/libsoem.so)\n"
    "Proceeding, but build will fail if these are missing."
  )
endif()

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SOEM_INCLUDE_DIR}
)

add_executable(azd_ked_pp_check
  src/azd_cia403.cpp
)

target_compile_options(azd_ked_pp_check PRIVATE -O2)
target_link_libraries(azd_ked_pp_check
  ${catkin_LIBRARIES}
  ${SOEM_LIB}
  pthread
)

#############
## Install ##
#############

install(TARGETS azd_ked_pp_check
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
