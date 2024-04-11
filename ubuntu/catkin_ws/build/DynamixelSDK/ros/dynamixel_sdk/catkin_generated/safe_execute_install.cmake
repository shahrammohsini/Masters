execute_process(COMMAND "/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/DynamixelSDK/ros/dynamixel_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/shahram/Documents/GitHub/Masters/ubuntu/catkin_ws/build/DynamixelSDK/ros/dynamixel_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
