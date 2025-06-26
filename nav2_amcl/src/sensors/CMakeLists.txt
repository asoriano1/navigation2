add_library(sensors_lib SHARED
  laser/laser.cpp
  laser/beam_model.cpp
  laser/likelihood_field_model.cpp
  laser/likelihood_field_model_prob.cpp
  intensity/likelihood_field_intensity_model.cpp
)

target_include_directories(sensors_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pf
)

target_link_libraries(sensors_lib pf_lib map_lib)
ament_target_dependencies(sensors_lib
  pluginlib
)


install(TARGETS
  sensors_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(sensors_lib HAS_LIBRARY_TARGET)
