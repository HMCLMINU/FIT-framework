# CMake generated Testfile for 
# Source directory: /home/autoware/Autoware/src/drivers/awf_drivers/as
# Build directory: /home/autoware/Autoware/build/as
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_as_roslint_package "/home/autoware/Autoware/build/as/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/autoware/Autoware/build/as/test_results/as/roslint-as.xml" "--working-dir" "/home/autoware/Autoware/build/as" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/autoware/Autoware/build/as/test_results/as/roslint-as.xml make roslint_as")
subdirs("gtest")
