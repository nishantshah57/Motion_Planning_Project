execute_process(COMMAND "/home/nishant/Motion_Planning_Project/mp_project/build/manipulation_and_grasping/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nishant/Motion_Planning_Project/mp_project/build/manipulation_and_grasping/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
