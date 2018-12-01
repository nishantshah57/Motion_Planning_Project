# Install script for directory: /home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nishant/Motion_Planning_Project/mp_project/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nishant/Motion_Planning_Project/mp_project/build/pr2_gazebo/catkin_generated/installspace/pr2_gazebo.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_gazebo/cmake" TYPE FILE FILES
    "/home/nishant/Motion_Planning_Project/mp_project/build/pr2_gazebo/catkin_generated/installspace/pr2_gazeboConfig.cmake"
    "/home/nishant/Motion_Planning_Project/mp_project/build/pr2_gazebo/catkin_generated/installspace/pr2_gazeboConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_gazebo" TYPE FILE FILES "/home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pr2_gazebo" TYPE PROGRAM FILES
    "/home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo/scripts/pr2_fingertip_pressure_contact_translator.py"
    "/home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo/scripts/pr2_simulate_torso_spring.py"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_gazebo/config" TYPE DIRECTORY FILES "/home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo/config/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_gazebo/launch" TYPE DIRECTORY FILES "/home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo/launch/")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pr2_gazebo/urdf" TYPE DIRECTORY FILES "/home/nishant/Motion_Planning_Project/mp_project/src/pr2_gazebo/urdf/")
endif()

