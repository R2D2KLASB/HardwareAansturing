# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2020.3.3\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2020.3.3\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Code_Ros_to_arduino.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Code_Ros_to_arduino.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Code_Ros_to_arduino.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Code_Ros_to_arduino.dir/flags.make

CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj: CMakeFiles/Code_Ros_to_arduino.dir/flags.make
CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj: ../RPI_send_string.cpp
CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj: CMakeFiles/Code_Ros_to_arduino.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj"
	C:\SOFTWARE_TI\i686-7.3.0-release-posix-dwarf-rt_v5-rev0\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj -MF CMakeFiles\Code_Ros_to_arduino.dir\RPI_send_string.cpp.obj.d -o CMakeFiles\Code_Ros_to_arduino.dir\RPI_send_string.cpp.obj -c C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\RPI_send_string.cpp

CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.i"
	C:\SOFTWARE_TI\i686-7.3.0-release-posix-dwarf-rt_v5-rev0\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\RPI_send_string.cpp > CMakeFiles\Code_Ros_to_arduino.dir\RPI_send_string.cpp.i

CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.s"
	C:\SOFTWARE_TI\i686-7.3.0-release-posix-dwarf-rt_v5-rev0\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\RPI_send_string.cpp -o CMakeFiles\Code_Ros_to_arduino.dir\RPI_send_string.cpp.s

CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj: CMakeFiles/Code_Ros_to_arduino.dir/flags.make
CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj: ../serialib-master/lib/serialib.cpp
CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj: CMakeFiles/Code_Ros_to_arduino.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj"
	C:\SOFTWARE_TI\i686-7.3.0-release-posix-dwarf-rt_v5-rev0\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj -MF CMakeFiles\Code_Ros_to_arduino.dir\serialib-master\lib\serialib.cpp.obj.d -o CMakeFiles\Code_Ros_to_arduino.dir\serialib-master\lib\serialib.cpp.obj -c C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\serialib-master\lib\serialib.cpp

CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.i"
	C:\SOFTWARE_TI\i686-7.3.0-release-posix-dwarf-rt_v5-rev0\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\serialib-master\lib\serialib.cpp > CMakeFiles\Code_Ros_to_arduino.dir\serialib-master\lib\serialib.cpp.i

CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.s"
	C:\SOFTWARE_TI\i686-7.3.0-release-posix-dwarf-rt_v5-rev0\mingw32\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\serialib-master\lib\serialib.cpp -o CMakeFiles\Code_Ros_to_arduino.dir\serialib-master\lib\serialib.cpp.s

# Object files for target Code_Ros_to_arduino
Code_Ros_to_arduino_OBJECTS = \
"CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj" \
"CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj"

# External object files for target Code_Ros_to_arduino
Code_Ros_to_arduino_EXTERNAL_OBJECTS =

Code_Ros_to_arduino.exe: CMakeFiles/Code_Ros_to_arduino.dir/RPI_send_string.cpp.obj
Code_Ros_to_arduino.exe: CMakeFiles/Code_Ros_to_arduino.dir/serialib-master/lib/serialib.cpp.obj
Code_Ros_to_arduino.exe: CMakeFiles/Code_Ros_to_arduino.dir/build.make
Code_Ros_to_arduino.exe: /home/deltarobot/clion/serialib-master/lib
Code_Ros_to_arduino.exe: CMakeFiles/Code_Ros_to_arduino.dir/linklibs.rsp
Code_Ros_to_arduino.exe: CMakeFiles/Code_Ros_to_arduino.dir/objects1.rsp
Code_Ros_to_arduino.exe: CMakeFiles/Code_Ros_to_arduino.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Code_Ros_to_arduino.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Code_Ros_to_arduino.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Code_Ros_to_arduino.dir/build: Code_Ros_to_arduino.exe
.PHONY : CMakeFiles/Code_Ros_to_arduino.dir/build

CMakeFiles/Code_Ros_to_arduino.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Code_Ros_to_arduino.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Code_Ros_to_arduino.dir/clean

CMakeFiles/Code_Ros_to_arduino.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug C:\Users\dhoeg\OneDrive\HBO_HU\HardwareAansturing\Code_Ros_to_arduino\cmake-build-debug\CMakeFiles\Code_Ros_to_arduino.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Code_Ros_to_arduino.dir/depend
