cmake_minimum_required(VERSION 3.0.2)
project(azd_ked_driver)

# C++11 以上
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)  # roscpp 等は使っていないので最小

# ===== SOEM (system) =====
# 例: apt でインストール済みの libsoem-dev などを想定。
# 見つからない場合は -DSOEM_INCLUDE_DIR=... -DSOEM_LIB=... で手動指定可。
find_path(SOEM_INCLUDE_DIR
  NAMES soem/ethercat.h
  PATHS /usr/include /usr/local/include
)
find_library(SOEM_LIB
  NAMES soem
  PATHS /usr/lib /usr/local/lib
)

if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIB)
  message(WARNING
    "SOEM not found automatically.\n"
    "  - Set SOEM_INCLUDE_DIR to the folder containing soem/ethercat.h\n"
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

# チェック用の main() を含む実行ファイル（元のソース名を使用）
add_executable(azd_ked_pp_check
  src/azd_cia403.cpp
)

# 最適化とスレッド（SOEMは送受信でpthreadが必要な環境が多い）
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
