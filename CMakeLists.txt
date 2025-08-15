cmake_minimum_required(VERSION 3.0.2)
project(azd_ked_driver)

# ===== Build settings =====
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Defaults used by the CLI tool when args are omitted
set(IF_DEFAULT    "enp0s31f6" CACHE STRING "Default NIC name when argv[1] is omitted")
set(DELTA_DEFAULT "20000"     CACHE STRING "Default delta (counts) when argv[2] is omitted")

# ===== Catkin / ROS =====
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Threads REQUIRED)

# ===== SOEM (system) =====
# Typical locations: /usr/local/include/soem/ethercat.h and /usr/local/lib/libsoem.so
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
message(STATUS "SOEM_INCLUDE_DIR = ${SOEM_INCLUDE_DIR}")
message(STATUS "SOEM_LIB         = ${SOEM_LIB}")

# Prepare a common include list; also add the parent of SOEM include so that both
# <ethercat.h> and <soem/ethercat.h> work.
set(_COMMON_INCLUDES
  include
  ${catkin_INCLUDE_DIRS}
)
if(SOEM_INCLUDE_DIR)
  list(APPEND _COMMON_INCLUDES ${SOEM_INCLUDE_DIR})
  get_filename_component(_SOEM_PARENT "${SOEM_INCLUDE_DIR}" DIRECTORY)
  if(_SOEM_PARENT)
    list(APPEND _COMMON_INCLUDES "${_SOEM_PARENT}")
  endif()
endif()

# ===== Catkin package =====
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

# ===== Executables =====
## 1) Minimal PP control CLI
add_executable(azd_ked_pp_check
  src/azd_cia403.cpp
)
target_include_directories(azd_ked_pp_check PRIVATE ${_COMMON_INCLUDES})
target_compile_definitions(azd_ked_pp_check PRIVATE
  IF_DEFAULT="${IF_DEFAULT}"
  DELTA_DEFAULT=${DELTA_DEFAULT}
)
target_compile_options(azd_ked_pp_check PRIVATE -O2)
target_link_libraries(azd_ked_pp_check
  ${catkin_LIBRARIES}
  ${SOEM_LIB}
  Threads::Threads
)

## 2) ROS node
add_executable(azd_ked_node
  src/azd_ked_node.cpp
)
target_include_directories(azd_ked_node PRIVATE ${_COMMON_INCLUDES})
target_compile_options(azd_ked_node PRIVATE -O2)
target_link_libraries(azd_ked_node
  ${catkin_LIBRARIES}
  ${SOEM_LIB}
  Threads::Threads
)

## 3) Multi-motor ROS node
add_executable(azd_ked_multi_node
  src/azd_ked_multi_node.cpp
)
target_include_directories(azd_ked_multi_node PRIVATE ${_COMMON_INCLUDES})
target_compile_options(azd_ked_multi_node PRIVATE -O2)
target_link_libraries(azd_ked_multi_node
  ${catkin_LIBRARIES}
  ${SOEM_LIB}
  Threads::Threads
)

# ===== Post-build capabilities (automatic setcap) =====
# After you add a sudoers rule (NOPASSWD for setcap on this binary),
# this will run without prompting and keep capabilities across rebuilds.
find_program(SETCAP_EXEC setcap /usr/sbin/setcap /sbin/setcap)
if(SETCAP_EXEC)
  add_custom_command(TARGET azd_ked_pp_check POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "→ setcap on $<TARGET_FILE:azd_ked_pp_check>"
    COMMAND sudo -n ${SETCAP_EXEC} cap_net_raw,cap_net_admin+eip $<TARGET_FILE:azd_ked_pp_check> || true
    VERBATIM
  )
  add_custom_command(TARGET azd_ked_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "→ setcap on $<TARGET_FILE:azd_ked_node>"
    COMMAND sudo -n ${SETCAP_EXEC} cap_net_raw,cap_net_admin+eip $<TARGET_FILE:azd_ked_node> || true
    VERBATIM
  )
  add_custom_command(TARGET azd_ked_multi_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "→ setcap on $<TARGET_FILE:azd_ked_multi_node>"
    COMMAND sudo -n ${SETCAP_EXEC} cap_net_raw,cap_net_admin+eip $<TARGET_FILE:azd_ked_multi_node> || true
    VERBATIM
  )
endif()

# ===== Install =====
install(TARGETS
  azd_ked_pp_check
  azd_ked_node
  azd_ked_multi_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Optionally install launch files if the directory exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
endif()

add_custom_command(TARGET azd_ked_pp_check POST_BUILD
  COMMAND sudo setcap cap_net_raw,cap_net_admin=eip $<TARGET_FILE:azd_ked_pp_check>
)

add_custom_command(TARGET azd_ked_node POST_BUILD
  COMMAND sudo setcap cap_net_raw,cap_net_admin=eip $<TARGET_FILE:azd_ked_node>
)

add_custom_command(TARGET azd_ked_multi_node POST_BUILD
  COMMAND sudo setcap cap_net_raw,cap_net_admin=eip $<TARGET_FILE:azd_ked_multi_node>
)
