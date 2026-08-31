cmake_minimum_required(VERSION 3.13)
project(p450_px4_gazebo_plugins LANGUAGES CXX)

if(NOT DEFINED P450_PX4_ROOT OR P450_PX4_ROOT STREQUAL "")
  message(FATAL_ERROR "P450_PX4_ROOT must name the validated PX4 checkout")
endif()

set(P450_GPS_SOURCE
  "Tools/sitl_gazebo/src/gazebo_gps_plugin.cpp")
set(P450_GPS_SOURCE_SHA256
  "128a20dc01e140ebba7e7c889cce5c729aa1a013bcafd3b272ea0360f7a42c75")
set(P450_PATCHED_GPS_SOURCE_SHA256
  "dc66bf5b1d19e350edcff2bcdd533c659f5836360a5f71562de41a4001ff304c")
set(P450_GPS_HEADER
  "Tools/sitl_gazebo/include/gazebo_gps_plugin.h")
set(P450_GPS_HEADER_SHA256
  "fcaa736262d89575a6084e659500cb8889d6a927c91f1c044eab5587e74f357f")
set(P450_GPS_GENERATED_HEADER
  "build/amovlab_sitl_default/build_gazebo/SITLGps.pb.h")
set(P450_GPS_GENERATED_HEADER_SHA256
  "537727dde21637fcd8cf630da2c49f8d7004f868638c23b05939b17a5908a434")

set(P450_GROUNDTRUTH_SOURCE
  "Tools/sitl_gazebo/src/gazebo_groundtruth_plugin.cpp")
set(P450_GROUNDTRUTH_SOURCE_SHA256
  "8dd9d6bd2c7619f38fa0104b730c8bf5653243d80762bf26ade0eec4b94266a3")
set(P450_PATCHED_GROUNDTRUTH_SOURCE_SHA256
  "7e1caae1e01dafbacd2b1e3ffb80efdfe36ace0c6b06cbeefd261d9c85d8aa97")
set(P450_GROUNDTRUTH_HEADER
  "Tools/sitl_gazebo/include/gazebo_groundtruth_plugin.h")
set(P450_GROUNDTRUTH_HEADER_SHA256
  "aabc2cee0d9ee378aa25f9b7d62d5d7239682c129c115e97cc56f4577f39761f")
set(P450_GROUNDTRUTH_GENERATED_HEADER
  "build/amovlab_sitl_default/build_gazebo/Groundtruth.pb.h")
set(P450_GROUNDTRUTH_GENERATED_HEADER_SHA256
  "80229d01b44dbe7bd07b36f3c24dc2bfdb5156c23fd9a573d2ce73514e1e374b")

set(P450_COMMON_HEADER "Tools/sitl_gazebo/include/common.h")
set(P450_COMMON_HEADER_SHA256
  "329133e49171793664184dd9c60f4f24305e280600d9a274286ca5ec2eb84241")
set(P450_SENSOR_MESSAGES_LIBRARY
  "build/amovlab_sitl_default/build_gazebo/libsensor_msgs.so")
set(P450_SENSOR_MESSAGES_LIBRARY_SHA256
  "203679a0bca52a1839a108ea3c66ab431004ee13a4f1e6b78775d549c74d6ce5")

set(P450_SNAPSHOT_ROOT "${CMAKE_CURRENT_BINARY_DIR}/snapshot")
set(P450_SNAPSHOT_INCLUDE "${P450_SNAPSHOT_ROOT}/include")
set(P450_SNAPSHOT_GENERATED "${P450_SNAPSHOT_ROOT}/generated")
set(P450_SNAPSHOT_SOURCE "${P450_SNAPSHOT_ROOT}/source")
set(P450_SNAPSHOT_LIBRARY "${P450_SNAPSHOT_ROOT}/lib")
file(MAKE_DIRECTORY
  "${P450_SNAPSHOT_INCLUDE}"
  "${P450_SNAPSHOT_GENERATED}"
  "${P450_SNAPSHOT_SOURCE}"
  "${P450_SNAPSHOT_LIBRARY}")

function(p450_snapshot INPUT_RELATIVE OUTPUT_PATH EXPECTED_SHA256 LABEL)
  set(P450_INPUT "${P450_PX4_ROOT}/${INPUT_RELATIVE}")
  if(NOT EXISTS "${P450_INPUT}")
    message(FATAL_ERROR "Required pinned PX4 artifact is missing: ${LABEL}")
  endif()
  # Compile and link only this SIM-owned snapshot.  The external checkout is
  # never consumed by Ninja after configure, closing the hash/build TOCTOU.
  configure_file("${P450_INPUT}" "${OUTPUT_PATH}" COPYONLY)
  file(SHA256 "${OUTPUT_PATH}" P450_OBSERVED_SHA256)
  if(NOT P450_OBSERVED_SHA256 STREQUAL EXPECTED_SHA256)
    message(FATAL_ERROR
      "${LABEL} SHA-256 mismatch: ${P450_OBSERVED_SHA256}")
  endif()
endfunction()

set(P450_SNAPSHOT_GPS_SOURCE
  "${P450_SNAPSHOT_SOURCE}/gazebo_gps_plugin.cpp")
set(P450_SNAPSHOT_GROUNDTRUTH_SOURCE
  "${P450_SNAPSHOT_SOURCE}/gazebo_groundtruth_plugin.cpp")
set(P450_SNAPSHOT_GPS_HEADER
  "${P450_SNAPSHOT_INCLUDE}/gazebo_gps_plugin.h")
set(P450_SNAPSHOT_GROUNDTRUTH_HEADER
  "${P450_SNAPSHOT_INCLUDE}/gazebo_groundtruth_plugin.h")
set(P450_SNAPSHOT_COMMON_HEADER "${P450_SNAPSHOT_INCLUDE}/common.h")
set(P450_SNAPSHOT_GPS_GENERATED
  "${P450_SNAPSHOT_GENERATED}/SITLGps.pb.h")
set(P450_SNAPSHOT_GROUNDTRUTH_GENERATED
  "${P450_SNAPSHOT_GENERATED}/Groundtruth.pb.h")
set(P450_SNAPSHOT_SENSOR_MESSAGES
  "${P450_SNAPSHOT_LIBRARY}/libsensor_msgs.so")

p450_snapshot("${P450_GPS_SOURCE}" "${P450_SNAPSHOT_GPS_SOURCE}"
  "${P450_GPS_SOURCE_SHA256}" "GPS source")
p450_snapshot("${P450_GPS_HEADER}" "${P450_SNAPSHOT_GPS_HEADER}"
  "${P450_GPS_HEADER_SHA256}" "GPS header")
p450_snapshot("${P450_GPS_GENERATED_HEADER}"
  "${P450_SNAPSHOT_GPS_GENERATED}"
  "${P450_GPS_GENERATED_HEADER_SHA256}" "GPS generated header")
p450_snapshot("${P450_GROUNDTRUTH_SOURCE}"
  "${P450_SNAPSHOT_GROUNDTRUTH_SOURCE}"
  "${P450_GROUNDTRUTH_SOURCE_SHA256}" "Groundtruth source")
p450_snapshot("${P450_GROUNDTRUTH_HEADER}"
  "${P450_SNAPSHOT_GROUNDTRUTH_HEADER}"
  "${P450_GROUNDTRUTH_HEADER_SHA256}" "Groundtruth header")
p450_snapshot("${P450_GROUNDTRUTH_GENERATED_HEADER}"
  "${P450_SNAPSHOT_GROUNDTRUTH_GENERATED}"
  "${P450_GROUNDTRUTH_GENERATED_HEADER_SHA256}"
  "Groundtruth generated header")
p450_snapshot("${P450_COMMON_HEADER}" "${P450_SNAPSHOT_COMMON_HEADER}"
  "${P450_COMMON_HEADER_SHA256}" "common header")
p450_snapshot("${P450_SENSOR_MESSAGES_LIBRARY}"
  "${P450_SNAPSHOT_SENSOR_MESSAGES}"
  "${P450_SENSOR_MESSAGES_LIBRARY_SHA256}" "sensor messages library")

function(p450_patch_plugin INPUT_SOURCE OUTPUT_SOURCE CONNECTION_BAD
    CONNECTION_GOOD EXPECTED_PATCHED_SHA256 LABEL)
  file(READ "${INPUT_SOURCE}" P450_SOURCE_TEXT)
  set(P450_UNSAFE_RESET "  world_->Reset();\n")
  string(LENGTH "${P450_SOURCE_TEXT}" P450_SOURCE_LENGTH_BEFORE)
  string(REPLACE "${P450_UNSAFE_RESET}" "" P450_PATCHED_SOURCE_TEXT
    "${P450_SOURCE_TEXT}")
  string(LENGTH "${P450_PATCHED_SOURCE_TEXT}" P450_SOURCE_LENGTH_AFTER_RESET)
  string(LENGTH "${P450_UNSAFE_RESET}" P450_RESET_LENGTH)
  math(EXPR P450_REMOVED_RESET_LENGTH
    "${P450_SOURCE_LENGTH_BEFORE} - ${P450_SOURCE_LENGTH_AFTER_RESET}")
  if(NOT P450_REMOVED_RESET_LENGTH EQUAL P450_RESET_LENGTH)
    message(FATAL_ERROR "${LABEL} must contain exactly one shutdown reset")
  endif()
  string(REPLACE "${CONNECTION_BAD}" "${CONNECTION_GOOD}"
    P450_PATCHED_SOURCE_TEXT "${P450_PATCHED_SOURCE_TEXT}")
  if(P450_PATCHED_SOURCE_TEXT MATCHES "world_->Reset\\(\\)" OR
      P450_PATCHED_SOURCE_TEXT MATCHES "->~Connection\\(\\)")
    message(FATAL_ERROR "${LABEL} still contains unsafe shutdown calls")
  endif()
  file(WRITE "${OUTPUT_SOURCE}" "${P450_PATCHED_SOURCE_TEXT}")
  file(SHA256 "${OUTPUT_SOURCE}" P450_OBSERVED_PATCHED_SHA256)
  if(NOT P450_OBSERVED_PATCHED_SHA256 STREQUAL EXPECTED_PATCHED_SHA256)
    message(FATAL_ERROR
      "Patched ${LABEL} SHA-256 mismatch: ${P450_OBSERVED_PATCHED_SHA256}")
  endif()
endfunction()

set(P450_PATCHED_GPS_SOURCE
  "${P450_SNAPSHOT_SOURCE}/patched_gazebo_gps_plugin.cpp")
set(P450_PATCHED_GROUNDTRUTH_SOURCE
  "${P450_SNAPSHOT_SOURCE}/patched_gazebo_groundtruth_plugin.cpp")
p450_patch_plugin(
  "${P450_SNAPSHOT_GPS_SOURCE}" "${P450_PATCHED_GPS_SOURCE}"
  "updateSensorConnection_->~Connection();"
  "updateSensorConnection_.reset();"
  "${P450_PATCHED_GPS_SOURCE_SHA256}" "GPS source")
p450_patch_plugin(
  "${P450_SNAPSHOT_GROUNDTRUTH_SOURCE}"
  "${P450_PATCHED_GROUNDTRUTH_SOURCE}"
  "updateConnection_->~Connection();" "updateConnection_.reset();"
  "${P450_PATCHED_GROUNDTRUTH_SOURCE_SHA256}" "Groundtruth source")

# The generated protobuf headers and libsensor_msgs.so are pinned PX4 build
# artifacts.  Do not regenerate protobuf: their exact ABI is the contract.
find_package(Protobuf 3.6.1 EXACT REQUIRED)
find_package(gazebo 11 REQUIRED)

set(CMAKE_SKIP_RPATH TRUE)
foreach(P450_PLUGIN IN ITEMS gps groundtruth)
  if(P450_PLUGIN STREQUAL "gps")
    set(P450_PLUGIN_SOURCE "${P450_PATCHED_GPS_SOURCE}")
  else()
    set(P450_PLUGIN_SOURCE "${P450_PATCHED_GROUNDTRUTH_SOURCE}")
  endif()
  set(P450_TARGET "gazebo_${P450_PLUGIN}_plugin")
  add_library("${P450_TARGET}" SHARED "${P450_PLUGIN_SOURCE}")
  target_compile_features("${P450_TARGET}" PRIVATE cxx_std_17)
  target_compile_options("${P450_TARGET}" PRIVATE
    -Wno-deprecated-declarations
    -Wno-address-of-packed-member)
  target_include_directories("${P450_TARGET}" PRIVATE
    "${P450_SNAPSHOT_INCLUDE}"
    "${P450_SNAPSHOT_GENERATED}"
    ${GAZEBO_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS})
  target_link_directories("${P450_TARGET}" PRIVATE ${GAZEBO_LIBRARY_DIRS})
  target_link_libraries("${P450_TARGET}" PRIVATE
    ${GAZEBO_LIBRARIES}
    ${Protobuf_LIBRARIES}
    "${P450_SNAPSHOT_SENSOR_MESSAGES}")
  set_target_properties("${P450_TARGET}" PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    INSTALL_RPATH ""
    INSTALL_RPATH_USE_LINK_PATH FALSE
    SKIP_BUILD_RPATH TRUE)
endforeach()

install(TARGETS gazebo_gps_plugin gazebo_groundtruth_plugin
  LIBRARY DESTINATION lib)
