add_rostest(flight_control.test ARGS headless:=true airframe:=bi)
add_rostest(flight_control.test ARGS headless:=true airframe:=tri)
add_rostest(flight_control.test ARGS headless:=true airframe:=tri_omni)
add_rostest(flight_control.test ARGS headless:=true airframe:=quad)

catkin_add_gtest(roll_yaw_core_test roll_yaw_core_test.cpp)
if(TARGET roll_yaw_core_test)
  target_link_libraries(roll_yaw_core_test gimbalrotor_perching ${catkin_LIBRARIES})
endif()
