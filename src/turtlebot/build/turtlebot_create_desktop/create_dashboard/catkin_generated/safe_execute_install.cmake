execute_process(COMMAND "/home/namwob44/turtlebot/build/turtlebot_create_desktop/create_dashboard/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/namwob44/turtlebot/build/turtlebot_create_desktop/create_dashboard/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
