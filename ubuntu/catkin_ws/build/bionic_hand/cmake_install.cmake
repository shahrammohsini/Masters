# Install script for directory: /home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bionic_hand/msg" TYPE FILE FILES
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/msg/SetPosition.msg"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/msg/SetPWM.msg"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/msg/FingerJoints.msg"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/msg/FingerPos.msg"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/msg/ControlCommands.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bionic_hand/srv" TYPE FILE FILES
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/srv/GetPosition.srv"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/srv/SyncGetPosition.srv"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/srv/BulkGetItem.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bionic_hand/cmake" TYPE FILE FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand/catkin_generated/installspace/bionic_hand-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/include/bionic_hand")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/share/roseus/ros/bionic_hand")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/share/common-lisp/ros/bionic_hand")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/share/gennodejs/ros/bionic_hand")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/python3/dist-packages/bionic_hand")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/devel/lib/python3/dist-packages/bionic_hand")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand/catkin_generated/installspace/bionic_hand.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bionic_hand/cmake" TYPE FILE FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand/catkin_generated/installspace/bionic_hand-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bionic_hand/cmake" TYPE FILE FILES
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand/catkin_generated/installspace/bionic_handConfig.cmake"
    "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/bionic_hand/catkin_generated/installspace/bionic_handConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bionic_hand" TYPE FILE FILES "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/src/bionic_hand/package.xml")
endif()

