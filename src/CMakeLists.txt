# flame_ros_nodelets
add_library(flame_ros_nodelets
  flame_nodelet.cc
  utils.h
  utils.cc
  ros_sensor_streams/tracked_image_stream.h
  ros_sensor_streams/tracked_image_stream.cc
  ros_sensor_streams/conversions.h
  )
target_link_libraries(flame_ros_nodelets
  ${flame_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES})
install(TARGETS flame_ros_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Needed to ensure flame_ros messages are built before these targets.
add_dependencies(flame_ros_nodelets
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if (FLAME_WITH_FLA)
  add_dependencies(flame_ros_nodelets fla_msgs_generate_messages_cpp)
endif (FLAME_WITH_FLA)

# flame_offline_tum.
add_executable(flame_offline_tum flame_offline_tum.cc
  utils.h
  utils.cc
  ros_sensor_streams/tum_rgbd_offline_stream.h
  ros_sensor_streams/tum_rgbd_offline_stream.cc)
target_link_libraries(flame_offline_tum
  ${flame_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES})
install(TARGETS flame_offline_tum
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Needed to ensure flame_ros messages are built before these targets.
add_dependencies(flame_offline_tum
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# flame_offline_asl.
add_executable(flame_offline_asl flame_offline_asl.cc
  utils.h
  utils.cc
  ros_sensor_streams/asl_rgbd_offline_stream.h
  ros_sensor_streams/asl_rgbd_offline_stream.cc
  dataset_utils/asl/dataset.h
  dataset_utils/asl/types.h
  dataset_utils/utils.h
  dataset_utils/utils.cc)
target_link_libraries(flame_offline_asl
  ${flame_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES})
install(TARGETS flame_offline_asl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Needed to ensure flame_ros messages are built before these targets.
add_dependencies(flame_offline_asl
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (FLAME_WITH_RVIZ)
  ## Here we specify which header files need to be run through "moc",
  ## Qt's meta-object compiler.
  set(MOC_FILES
    flame_rviz_plugins/textured_mesh_display.h)

  ## Declare a cpp library
  # add_library(flame_rviz_plugins
  #   flame_rviz_plugins/flame_rviz_plugins.cpp
  # )
  add_library(flame_rviz_plugins
    flame_rviz_plugins/visual.h
    flame_rviz_plugins/textured_mesh_display.cc
    flame_rviz_plugins/textured_mesh_visual.h
    flame_rviz_plugins/textured_mesh_visual.cc
    flame_rviz_plugins/surface_normals_visual.h
    flame_rviz_plugins/surface_normals_visual.cc
    ${MOC_FILES})

  ## Add cmake target dependencies of the executable/library
  ## as an example, message headers may need to be generated before nodes
  # add_dependencies(flame_rviz_plugins_node flame_rviz_plugins_generate_messages_cpp)
  add_dependencies(flame_rviz_plugins
    pcl_msgs_generate_messages_cpp)

  ## Specify libraries to link a library or executable target against
  # target_link_libraries(flame_rviz_plugins_node
  #   ${catkin_LIBRARIES}
  # )
  target_link_libraries(flame_rviz_plugins
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS})

  install(TARGETS flame_rviz_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif ()
