include_directories(${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/include)
set(ros_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/lib/librosconsole.so
        ${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/lib/libroslz4.so
        ${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/lib/libroscpp.so
        ${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/lib/librostime.so
        ${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/lib/librosconsole_bridge.so
        ${CMAKE_CURRENT_SOURCE_DIR}/opt/ros/kinetic/lib/libroscpp_serialization.so
        ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libboost_system.so
        ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/liblog4cxx.so
        ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libboost_regex.so
        ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libboost_regex.so
        ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libapr-1.so
        ${CMAKE_CURRENT_SOURCE_DIR}/usr/lib/arm-linux-gnueabihf/libaprutil-1.so
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm-linux-gnueabihf/libexpat.so.1.6.0
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm-linux-gnueabihf/libuuid.so.1
        )
message(STATUS "ros_LIBRARIES: ${ros_LIBRARIES}" )