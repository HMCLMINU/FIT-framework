# CMake generated Testfile for 
# Source directory: /home/autoware/Autoware/src/autoware/common/libvectormap
# Build directory: /home/autoware/Autoware/build/libvectormap
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_libvectormap_roslint_package "/home/autoware/Autoware/build/libvectormap/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/autoware/Autoware/build/libvectormap/test_results/libvectormap/roslint-libvectormap.xml" "--working-dir" "/home/autoware/Autoware/build/libvectormap" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/autoware/Autoware/build/libvectormap/test_results/libvectormap/roslint-libvectormap.xml make roslint_libvectormap")
subdirs("gtest")
