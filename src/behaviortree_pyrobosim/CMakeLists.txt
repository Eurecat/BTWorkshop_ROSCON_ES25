cmake_minimum_required(VERSION 3.8)
project(behaviortree_pyrobosim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPS
  behaviortree_ros2
  simple_pyrobosim_msgs
  )

# find dependencies
foreach(PACKAGE ${THIS_PACKAGE_DEPS})
    find_package(${PACKAGE} REQUIRED )
endforeach()


add_library(${PROJECT_NAME}_nodemodels SHARED src/plugin.cpp)
target_include_directories(${PROJECT_NAME}_nodemodels PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}_nodemodels PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(${PROJECT_NAME}_nodemodels ${THIS_PACKAGE_DEPS})

######################################################
# INSTALL

# Recover subfolder path for plugins
execute_process(
    COMMAND bash -c "xmllint --xpath 'string(//description/plugins/@folder_path)' plugins_description.xml | awk '{print \$1\}'"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE PLUGINS_FOLDER_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND bash -c "xmllint --xpath 'string(//description/palettes/@folder_path)' palettes_description.xml | awk '{print \$1\}'"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE PALETTES_FOLDER_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

install(TARGETS
  ${PROJECT_NAME}_nodemodels
  LIBRARY DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  ARCHIVE DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
  RUNTIME DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH}
)

# Assume behaviortree_eut_pluginsConfig.cmake is installed into <install_prefix>/share/behaviortree_eut_plugins.
# Get the install prefix from the config file location:
get_filename_component(_bt_ros2_share_intern_dir "${behaviortree_ros2_DIR}" DIRECTORY)
get_filename_component(_bt_ros2_share_dir "${_bt_ros2_share_intern_dir}" DIRECTORY)
get_filename_component(_bt_ros2_install_dir "${_bt_ros2_share_dir}" DIRECTORY)

# Construct the expected path to the bt_plugins_to_xml executable:
set(BT_PLUGINS_TO_XML_EXECUTABLE "${_bt_ros2_install_dir}/lib/behaviortree_ros2/bt_ros_plugins_to_xml")

if(NOT EXISTS "${BT_PLUGINS_TO_XML_EXECUTABLE}")
  message(FATAL_ERROR "bt_ros_plugins_to_xml executable not found. Ensure behaviortree_ros2 is built and installed.")
endif()

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
  COMMAND ${BT_PLUGINS_TO_XML_EXECUTABLE}
          $<TARGET_FILE:${PROJECT_NAME}_nodemodels>
          ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
  DEPENDS ${PROJECT_NAME}_nodemodels
  COMMENT "Generating XML file from plugin .so"
)

add_custom_target(generate_xml ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
)

add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor  ${THIS_PACKAGE_DEPS})

install(TARGETS
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libbt_${PROJECT_NAME}_nodemodels.xml
        DESTINATION share/${PROJECT_NAME}/${PLUGINS_FOLDER_PATH})

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Register the plugin description file as a resource
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/plugins_description.xml
               ${CMAKE_CURRENT_BINARY_DIR}/plugins_description.xml COPYONLY)

ament_index_register_resource(
  "behavior_tree_plugin"
  CONTENT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/plugins_description.xml"
)

# Register the palette description file as a resource
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/palettes_description.xml
               ${CMAKE_CURRENT_BINARY_DIR}/palettes_description.xml COPYONLY)

ament_index_register_resource(
  "behavior_tree_palette"
  CONTENT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/palettes_description.xml"
)

ament_package()
