# Meta
set(AM_MULTI_CONFIG "SINGLE")
# Directories and files
set(AM_CMAKE_BINARY_DIR "/home/autoware/Autoware/build/trafficlight_recognizer/")
set(AM_CMAKE_SOURCE_DIR "/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/")
set(AM_CMAKE_CURRENT_SOURCE_DIR "/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/")
set(AM_CMAKE_CURRENT_BINARY_DIR "/home/autoware/Autoware/build/trafficlight_recognizer/")
set(AM_CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE "")
set(AM_BUILD_DIR "/home/autoware/Autoware/build/trafficlight_recognizer/label_maker_autogen")
set(AM_SOURCES "/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/nodes/label_maker/custom_graphics_view.cpp;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/nodes/label_maker/file_system_operator.cpp;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/nodes/label_maker/label_maker.cpp;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/nodes/label_maker/label_maker_gui.cpp")
set(AM_HEADERS "/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/include/trafficlight_recognizer/label_maker/custom_graphics_view.h;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/include/trafficlight_recognizer/label_maker/file_system_operator.h;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/include/trafficlight_recognizer/label_maker/label_maker_gui.h")
# Qt environment
set(AM_QT_VERSION_MAJOR "5")
set(AM_QT_VERSION_MINOR "9")
set(AM_QT_MOC_EXECUTABLE "/usr/lib/qt5/bin/moc")
set(AM_QT_UIC_EXECUTABLE "/usr/lib/qt5/bin/uic")
set(AM_QT_RCC_EXECUTABLE "/usr/lib/qt5/bin/rcc")
# MOC settings
set(AM_MOC_SKIP "/home/autoware/Autoware/build/trafficlight_recognizer/feat_proj_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/feat_proj_lanelet2_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/label_maker_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/libcontext_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/region_tlr_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/roi_extractor_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/test-feat_proj_lanelet2_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/tl_switch_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/tlr_tuner_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/ui_label_maker_gui.h;/home/autoware/Autoware/build/trafficlight_recognizer/ui_mainwindow.h")
set(AM_MOC_DEFINITIONS "QT_CORE_LIB;QT_GUI_LIB;QT_NO_DEBUG;QT_WIDGETS_LIB;ROSCONSOLE_BACKEND_LOG4CXX;ROS_BUILD_SHARED_LIBS=1;ROS_PACKAGE_NAME=\"trafficlight_recognizer\"")
set(AM_MOC_INCLUDES "/home/autoware/Autoware/build/trafficlight_recognizer;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer;/home/autoware/Autoware/build/trafficlight_recognizer/label_maker_autogen/include;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/include;/home/autoware/Autoware/install/vector_map_server/include;/home/autoware/Autoware/install/libvectormap/include;/home/autoware/Autoware/install/lanelet2_extension/include;/home/autoware/Autoware/install/vector_map/include;/home/autoware/Autoware/install/amathutils_lib/include;/home/autoware/Autoware/install/vector_map_msgs/include;/home/autoware/Autoware/install/autoware_msgs/include;/home/autoware/Autoware/install/autoware_lanelet2_msgs/include;/opt/ros/melodic/include;/opt/ros/melodic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp;/usr/include/opencv;/usr/local/include/eigen3;/usr/include/x86_64-linux-gnu/qt5;/usr/include/x86_64-linux-gnu/qt5/QtCore;/usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++;/usr/include/x86_64-linux-gnu/qt5/QtGui;/usr/include/x86_64-linux-gnu/qt5/QtWidgets;/usr/include")
set(AM_MOC_OPTIONS "")
set(AM_MOC_RELAXED_MODE "FALSE")
set(AM_MOC_MACRO_NAMES "Q_OBJECT;Q_GADGET;Q_NAMESPACE")
set(AM_MOC_DEPEND_FILTERS "")
set(AM_MOC_PREDEFS_CMD "/usr/bin/c++;-dM;-E;-c;/usr/share/cmake-3.10/Modules/CMakeCXXCompilerABI.cpp")
# UIC settings
set(AM_UIC_SKIP "/home/autoware/Autoware/build/trafficlight_recognizer/feat_proj_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/feat_proj_lanelet2_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/label_maker_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/libcontext_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/region_tlr_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/roi_extractor_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/test-feat_proj_lanelet2_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/tl_switch_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/tlr_tuner_autogen/mocs_compilation.cpp;/home/autoware/Autoware/build/trafficlight_recognizer/ui_label_maker_gui.h;/home/autoware/Autoware/build/trafficlight_recognizer/ui_mainwindow.h;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/nodes/label_maker/label_maker_gui.ui;/home/autoware/Autoware/src/autoware/core_perception/trafficlight_recognizer/nodes/tlr_tuner/mainwindow.ui")
set(AM_UIC_TARGET_OPTIONS "")
set(AM_UIC_OPTIONS_FILES "")
set(AM_UIC_OPTIONS_OPTIONS "")
set(AM_UIC_SEARCH_PATHS "")
# RCC settings
set(AM_RCC_SOURCES "")
set(AM_RCC_BUILDS "")
set(AM_RCC_OPTIONS "")
set(AM_RCC_INPUTS "")
# Configurations options
set(AM_CONFIG_SUFFIX_Release "_Release")