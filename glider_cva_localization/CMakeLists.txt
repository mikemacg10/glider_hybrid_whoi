cmake_minimum_required(VERSION 3.0.2)
project(glider_cva_localization)

find_package(catkin REQUIRED)
catkin_package(
)
include_directories(
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_install_python(PROGRAMS scripts/Localizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)