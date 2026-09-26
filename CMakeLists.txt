# bilateral_arc_clearance_controller: arc-clearance obstacle avoidance.
#
# Layout follows nav2 controller plugin conventions. Builds in two modes:
#  - ament/colcon (ROS 2 workspace): core library + cmd_vel filter node +
#    nav2 controller plugin.
#  - Plain CMake (add_subdirectory or -S .): core library + scenario harness
#    only (no ROS dependency).
cmake_minimum_required(VERSION 3.16)
project(bilateral_arc_clearance_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Framework-free algorithm core
add_library(bac_core STATIC
    src/ackermann_motion_model.cpp
    src/adapter_utils.cpp
    src/arc_trajectory_evaluator.cpp
    src/bac_core.cpp
    src/diff_drive_motion_model.cpp
    src/motion_model.cpp
    src/omni_motion_model.cpp
)
target_include_directories(bac_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_options(bac_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(bac_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  option(BAC_BUILD_NAV2_PLUGIN
      "Build the Nav2 controller plugin (requires the supported Nav2 API)" ON)

  # Headers go to include/<package>/<package>/, the layout ROS 2 recommends
  # from Humble on: in a merged install space a package-specific include
  # directory cannot shadow another package's headers.
  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
  install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
      PATTERN "__pycache__" EXCLUDE
      PATTERN "*.pyc" EXCLUDE
  )
  # Installed because the installed documentation LINKS INTO THEM. README.md,
  # README.ja.md, docs/README.md, docs/en/README.md and both language versions
  # of docs/ablation_and_matched_evaluation.md carry 12 relative links to
  # docker/nav2/README.md and examples/gazebo/README.md between them; all
  # 12 resolve in the source tree and, until these two rules existed, none of
  # them resolved under share/${PROJECT_NAME} (measured by installing and
  # walking every relative link in every installed .md and .rst). Rewriting the
  # links as absolute repository URLs would have fixed the same 12, but would
  # point a user of an installed copy at whatever `main` holds rather than at
  # the harness that shipped with it.
  #
  # Cost, and SAY WHICH MEASURE, because they differ by more than a factor of
  # two. `du -sk` on the two installed directories: docker 28 kB +
  # examples 76 kB = 104 kB, against docs/ at 4292 kB by the same command.
  # Installing twice into a prefix, with these two rules and without, moves
  # share/${PROJECT_NAME} by the same +104 kB (4476 -> 4580 here; the absolute
  # figures depend on the build options, the delta does not). The 14 installed
  # files are 42387 bytes of actual content - the rest is 4 kiB block
  # granularity over 14 files and 7 directories.
  #
  # An earlier revision of this comment said 132 kB and could not be
  # reproduced. That figure was `du -sk` on the SOURCE tree - the wrong tree,
  # and one that still holds the 28 kB of
  # examples/gazebo/scripts/__pycache__ these rules exclude. 132 - 28 = 104.
  #
  # If either directory stops being installed, un-link it from the
  # documentation in the same change.
  install(DIRECTORY docker DESTINATION share/${PROJECT_NAME}
      PATTERN "__pycache__" EXCLUDE
      PATTERN "*.pyc" EXCLUDE
  )
  install(DIRECTORY examples DESTINATION share/${PROJECT_NAME}
      PATTERN "__pycache__" EXCLUDE
      PATTERN "*.pyc" EXCLUDE
  )
  install(FILES README.md README.ja.md LICENSE CHANGELOG.rst CONTRIBUTING.md SECURITY.md
      DESTINATION share/${PROJECT_NAME})
  install(TARGETS bac_core
      EXPORT export_bac_core
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
  )
  ament_export_targets(export_bac_core HAS_LIBRARY_TARGET)
  ament_export_include_directories("include/${PROJECT_NAME}")

  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  # ROS dependencies are linked as the CMake targets their packages export,
  # not through ament_target_dependencies(). That macro is deprecated from
  # Kilted on, and in Lyrical find_package(ament_cmake) no longer defines it,
  # so a package that still calls it fails to configure there. Message
  # packages export their targets as <pkg>_TARGETS on every distribution this
  # package builds on (Jazzy, Kilted, Lyrical); the rest export <pkg>::<name>.

  # Evaluation node: cmd_vel filter driven by a laser scan. Built as the
  # composable node bac::BacFilterNode, which a component container can load;
  # rclcpp_components generates the standalone bac_filter_node executable
  # (same name, same `ros2 run` / launch usage as before).
  add_library(bac_filter_component SHARED src/bac_filter_node.cpp)
  target_link_libraries(bac_filter_component PUBLIC
      bac_core
      rclcpp::rclcpp
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
  )
  target_link_libraries(bac_filter_component PRIVATE rclcpp_components::component)
  target_compile_options(bac_filter_component PRIVATE -Wall -Wextra -Wpedantic)
  rclcpp_components_register_node(bac_filter_component
      PLUGIN "bac::BacFilterNode"
      EXECUTABLE bac_filter_node
  )
  install(TARGETS bac_filter_component
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
  )

  if(BAC_BUILD_NAV2_PLUGIN)
    find_package(rclcpp_lifecycle REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(nav2_core REQUIRED)
    find_package(nav2_costmap_2d REQUIRED)
    find_package(nav2_util REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)

    # Which nav2_core::Controller interface to implement. Nav2 1.5 (ROS 2
    # Lyrical) changed it: the parent node is a nav2::LifecycleNode, setPlan()
    # became newPathReceived(), and computeVelocityCommands() also receives the
    # controller server's transformed plan and the goal. Nav2 1.3 (Jazzy) and
    # 1.4 (Kilted) share the earlier interface. The version is the one
    # nav2_core's package manifest reports through find_package(); BAC_NAV2_API
    # overrides the detection for a source build that reports none.
    set(BAC_NAV2_API "AUTO" CACHE STRING
        "nav2_core::Controller interface: AUTO, 1 (Nav2 <= 1.4, Jazzy/Kilted) or 2 (Nav2 >= 1.5, Lyrical)")
    if(BAC_NAV2_API STREQUAL "AUTO")
      if(NOT nav2_core_VERSION)
        message(FATAL_ERROR
            "nav2_core reports no version, so the nav2_core::Controller interface cannot be "
            "detected. Configure with -DBAC_NAV2_API=1 (Nav2 <= 1.4) or -DBAC_NAV2_API=2 (Nav2 >= 1.5).")
      elseif(nav2_core_VERSION VERSION_GREATER_EQUAL 1.5.0)
        set(_bac_nav2_api 2)
      else()
        set(_bac_nav2_api 1)
      endif()
    elseif(BAC_NAV2_API STREQUAL "1" OR BAC_NAV2_API STREQUAL "2")
      set(_bac_nav2_api ${BAC_NAV2_API})
    else()
      message(FATAL_ERROR "BAC_NAV2_API must be AUTO, 1 or 2, got '${BAC_NAV2_API}'")
    endif()
    message(STATUS "nav2_core ${nav2_core_VERSION}: building for nav2_core::Controller interface ${_bac_nav2_api}")

    # Nav2 controller plugin (built and tested against ROS 2 Jazzy and Lyrical)
    add_library(bilateral_arc_clearance_controller SHARED src/bac_controller.cpp)
    # PUBLIC: the adapter test includes the plugin header, which selects its
    # base-class signatures on this definition.
    target_compile_definitions(bilateral_arc_clearance_controller PUBLIC BAC_NAV2_API=${_bac_nav2_api})
    target_compile_options(bilateral_arc_clearance_controller PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(bilateral_arc_clearance_controller PUBLIC
        bac_core
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        pluginlib::pluginlib
        nav2_costmap_2d::nav2_costmap_2d_core
        nav2_util::nav2_util_core
        tf2::tf2
        tf2_geometry_msgs::tf2_geometry_msgs
        tf2_ros::tf2_ros
        ${diagnostic_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
    )
    # nav2_core is header-only. Jazzy exports only its include directory;
    # Kilted and Lyrical export an INTERFACE target that carries the
    # dependencies the headers need (in Lyrical, nav2_ros_common among them).
    if(TARGET nav2_core::nav2_core)
      target_link_libraries(bilateral_arc_clearance_controller PUBLIC nav2_core::nav2_core)
    else()
      target_include_directories(bilateral_arc_clearance_controller PUBLIC ${nav2_core_INCLUDE_DIRS})
    endif()
    if(_bac_nav2_api EQUAL 2)
      # nav2::LifecycleNode and nav2::TransformBuffer, the types interface 2
      # names in its signatures.
      find_package(nav2_ros_common REQUIRED)
      target_link_libraries(bilateral_arc_clearance_controller PUBLIC nav2_ros_common::nav2_ros_common)
    endif()
    install(TARGETS bilateral_arc_clearance_controller
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
    pluginlib_export_plugin_description_file(nav2_core bilateral_arc_clearance_controller_plugin.xml)
  endif()
endif()

# Closed-loop scenario harness (framework-free; also the package's test)
include(CTest)
if(BUILD_TESTING)
  add_executable(bac_core_unit test/core_unit.cpp)
  target_link_libraries(bac_core_unit bac_core)
  target_compile_options(bac_core_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacCoreUnit COMMAND bac_core_unit)
  # The suite includes inputs that used to hang process(); a regression must
  # fail here rather than stall CI until its job timeout.
  set_tests_properties(BacCoreUnit PROPERTIES TIMEOUT 120)

  # Differential-drive output reachability stage (behaviour every existing
  # differential-drive user receives; see CHANGELOG "Forthcoming").
  add_executable(bac_output_stage_unit test/output_stage_unit.cpp)
  target_link_libraries(bac_output_stage_unit bac_core)
  target_compile_options(bac_output_stage_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacOutputStageUnit COMMAND bac_output_stage_unit)
  set_tests_properties(BacOutputStageUnit PROPERTIES LABELS "regression;diff_drive")

  add_executable(bac_diff_drive_motion_model_unit test/diff_drive_motion_model_unit.cpp)
  target_include_directories(bac_diff_drive_motion_model_unit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bac_diff_drive_motion_model_unit bac_core)
  target_compile_options(bac_diff_drive_motion_model_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacDiffDriveMotionModelUnit COMMAND bac_diff_drive_motion_model_unit)

  add_executable(bac_omni_motion_model_unit test/omni_motion_model_unit.cpp)
  target_include_directories(bac_omni_motion_model_unit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bac_omni_motion_model_unit bac_core)
  target_compile_options(bac_omni_motion_model_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacOmniMotionModelUnit COMMAND bac_omni_motion_model_unit)
  set_tests_properties(BacOmniMotionModelUnit PROPERTIES LABELS "omni")

  add_executable(bac_ackermann_motion_model_unit test/ackermann_motion_model_unit.cpp)
  target_include_directories(bac_ackermann_motion_model_unit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(bac_ackermann_motion_model_unit bac_core)
  target_compile_options(bac_ackermann_motion_model_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacAckermannMotionModelUnit COMMAND bac_ackermann_motion_model_unit)
  set_tests_properties(BacAckermannMotionModelUnit PROPERTIES LABELS "ackermann")

  add_executable(bac_omni_scenarios test/omni_scenarios.cpp)
  target_include_directories(bac_omni_scenarios PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  # The suite runs the shipped example configuration itself, so a yaml edit
  # cannot ship a configuration that fails this suite.
  target_compile_definitions(bac_omni_scenarios PRIVATE
      BAC_OMNI_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/bac_controller_omni.yaml")
  target_link_libraries(bac_omni_scenarios bac_core)
  target_compile_options(bac_omni_scenarios PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacOmniScenarios COMMAND bac_omni_scenarios)
  set_tests_properties(BacOmniScenarios PROPERTIES LABELS "regression;omni")

  add_executable(bac_ackermann_scenarios test/ackermann_scenarios.cpp)
  target_include_directories(bac_ackermann_scenarios PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  # The suite runs the shipped example configuration itself, so a yaml edit
  # cannot ship a configuration that fails this suite.
  target_compile_definitions(bac_ackermann_scenarios PRIVATE
      BAC_ACKERMANN_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/bac_controller_ackermann.yaml")
  target_link_libraries(bac_ackermann_scenarios bac_core)
  target_compile_options(bac_ackermann_scenarios PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacAckermannScenarios COMMAND bac_ackermann_scenarios)
  set_tests_properties(BacAckermannScenarios PROPERTIES LABELS "regression;ackermann")

  add_executable(bac_adapter_utils_unit test/adapter_utils_unit.cpp)
  target_link_libraries(bac_adapter_utils_unit bac_core)
  target_compile_options(bac_adapter_utils_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacAdapterUtilsUnit COMMAND bac_adapter_utils_unit)

  add_executable(bac_perf_benchmark test/perf_benchmark.cpp)
  target_link_libraries(bac_perf_benchmark bac_core)
  target_compile_options(bac_perf_benchmark PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(bac_scenario_harness test/scenarios.cpp)
  target_include_directories(bac_scenario_harness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  # The harness guards the shipped DIFFERENTIAL-DRIVE configuration - the file
  # users copy first, and until this define existed the one shipped yaml no
  # suite could fail. Structure only: the scenarios below run bac::Params
  # defaults, not these values. See checkShippedDiffDriveConfig().
  target_compile_definitions(bac_scenario_harness PRIVATE
      BAC_DIFF_DRIVE_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/bac_controller.yaml")
  target_link_libraries(bac_scenario_harness bac_core)
  target_compile_options(bac_scenario_harness PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacScenarioHarness COMMAND bac_scenario_harness --strict)
  # A SELECTION HANDLE, not extra coverage. Same binary as BacScenarioHarness
  # with `--filter goal_` added, so it can never kill anything the unfiltered
  # run does not - measured over the 52 mutations of src/ and bac_core.hpp the
  # suite is scored against: it fails under emerg_normalized_lat_removed and
  # geo_sigma_flip, and BacScenarioHarness fails under both as well, so its
  # unique kill count is 0. It exists so that CI (and `ctest -L
  # goal_direction`) can run the goal-direction scenarios on their own; at
  # 0.05 s against the full harness's 2.44 s that costs nothing. Do not delete
  # it for killing nothing, and do not expect it to kill anything.
  add_test(
      NAME BacGoalDirectionRegression
      COMMAND bac_scenario_harness --strict --filter goal_
  )
  set_tests_properties(BacGoalDirectionRegression PROPERTIES
      LABELS "regression;goal_direction"
  )

  if(ament_cmake_FOUND)
    if(BAC_BUILD_NAV2_PLUGIN)
      add_executable(bac_ros_adapter_unit test/ros_adapter_unit.cpp)
      # The plugin links its ROS dependencies PUBLIC, so this carries them all.
      target_link_libraries(bac_ros_adapter_unit PRIVATE bilateral_arc_clearance_controller)
      target_compile_options(bac_ros_adapter_unit PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME BacRosAdapterUnit COMMAND bac_ros_adapter_unit)
    endif()

    # The filter node in-process: clock source, tick period, stamped and
    # unstamped cmd_vel, and refused parameter changes.
    find_package(rosgraph_msgs REQUIRED)
    add_executable(bac_filter_node_unit test/filter_node_unit.cpp)
    target_include_directories(bac_filter_node_unit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(bac_filter_node_unit PRIVATE bac_filter_component ${rosgraph_msgs_TARGETS})
    target_compile_options(bac_filter_node_unit PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME BacFilterNodeUnit COMMAND bac_filter_node_unit)
  endif()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
