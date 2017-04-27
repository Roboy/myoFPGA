# Install script for directory: /home/roboy/Downloads/openPowerLink/stack

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/roboy/Downloads/openPowerLink/stack/lib/linux/x86_64")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkmn/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkmnapp-userintf/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkmnapp-kernelintf/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkmnapp-kernelpcp/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkmndrv-pcap/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkmn-sim/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkcn/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkcnapp-userintf/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkcnapp-kernelintf/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkcndrv-pcap/cmake_install.cmake")
  include("/home/roboy/Downloads/openPowerLink/stack/build/linux/proj/linux/liboplkcn-sim/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/roboy/Downloads/openPowerLink/stack/build/linux/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
