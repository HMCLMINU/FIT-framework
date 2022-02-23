# CMake generated Testfile for 
# Source directory: /home/autoware/Autoware/src/autoware/common/vector_map_server
# Build directory: /home/autoware/Autoware/build/vector_map_server
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_vector_map_server_roslint_package "/home/autoware/Autoware/build/vector_map_server/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/autoware/Autoware/build/vector_map_server/test_results/vector_map_server/roslint-vector_map_server.xml" "--working-dir" "/home/autoware/Autoware/build/vector_map_server" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/autoware/Autoware/build/vector_map_server/test_results/vector_map_server/roslint-vector_map_server.xml make roslint_vector_map_server")
subdirs("gtest")
