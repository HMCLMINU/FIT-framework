# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/autoware/Autoware/build/adi_driver

# Utility rule file for debbuild_adi_driver.

# Include the progress variables for this target.
include CMakeFiles/debbuild_adi_driver.dir/progress.make

CMakeFiles/debbuild_adi_driver:
	cd /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices && sudo -n true || ( >&2 echo debbuild\ need\ sudo\ access\ to\ make\ debs && exit 1 )
	cd /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices && bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
	cd /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices && MAKEFLAGS= fakeroot debian/rules binary
	cd /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices && find debian obj-x86_64-linux-gnu -type f -not -name postinst -print0 | xargs -0 rm --
	cd /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices && find debian obj-x86_64-linux-gnu -type d -empty -delete

debbuild_adi_driver: CMakeFiles/debbuild_adi_driver
debbuild_adi_driver: CMakeFiles/debbuild_adi_driver.dir/build.make

.PHONY : debbuild_adi_driver

# Rule to build all files generated by this target.
CMakeFiles/debbuild_adi_driver.dir/build: debbuild_adi_driver

.PHONY : CMakeFiles/debbuild_adi_driver.dir/build

CMakeFiles/debbuild_adi_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/debbuild_adi_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/debbuild_adi_driver.dir/clean

CMakeFiles/debbuild_adi_driver.dir/depend:
	cd /home/autoware/Autoware/build/adi_driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices /home/autoware/Autoware/src/drivers/awf_drivers/analog_devices /home/autoware/Autoware/build/adi_driver /home/autoware/Autoware/build/adi_driver /home/autoware/Autoware/build/adi_driver/CMakeFiles/debbuild_adi_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/debbuild_adi_driver.dir/depend

