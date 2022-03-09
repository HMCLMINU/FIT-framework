# CMake generated Testfile for 
# Source directory: /home/autoware/Autoware/src/autoware/common/tvm_utility
# Build directory: /home/autoware/Autoware/build/tvm_utility
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_tvm_utility_roslint_package "/home/autoware/Autoware/build/tvm_utility/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/autoware/Autoware/build/tvm_utility/test_results/tvm_utility/roslint-tvm_utility.xml" "--working-dir" "/home/autoware/Autoware/build/tvm_utility" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/autoware/Autoware/build/tvm_utility/test_results/tvm_utility/roslint-tvm_utility.xml make roslint_tvm_utility")
subdirs("gtest")
