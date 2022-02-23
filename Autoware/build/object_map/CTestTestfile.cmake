# CMake generated Testfile for 
# Source directory: /home/autoware/Autoware/src/autoware/common/object_map
# Build directory: /home/autoware/Autoware/build/object_map
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_object_map_roslint_package "/home/autoware/Autoware/build/object_map/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/autoware/Autoware/build/object_map/test_results/object_map/roslint-object_map.xml" "--working-dir" "/home/autoware/Autoware/build/object_map" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/autoware/Autoware/build/object_map/test_results/object_map/roslint-object_map.xml make roslint_object_map")
subdirs("gtest")
