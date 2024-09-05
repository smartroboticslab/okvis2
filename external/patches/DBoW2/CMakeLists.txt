cmake_minimum_required(VERSION 3.8...3.16)
project(DBoW2)

set(DEPENDENCY_DIR ${CMAKE_CURRENT_BINARY_DIR}/dependencies)
set(DEPENDENCY_INSTALL_DIR ${DEPENDENCY_DIR}/install)

find_package(OpenCV REQUIRED COMPONENTS core)

add_library(${PROJECT_NAME}
  src/BowVector.cpp
  src/FBrief.cpp
  src/FORB.cpp
  src/FeatureVector.cpp
  src/QueryResults.cpp
  src/ScoringObject.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/DBoW2>
    $<INSTALL_INTERFACE:include/DBoW2>
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

install(TARGETS ${PROJECT_NAME} EXPORT DBoW2Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/DBoW2 DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")

# The directory where the .cmake files will be installed
include(GNUInstallDirs)
if(WIN32 AND NOT CYGWIN)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}/cmake")
elseif(APPLE)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}.framework/Resources/CMake")
else()
    set(INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PACKAGE_NAME}")
endif()
# Generate DBoW2Config.cmake
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/src/DBoW2.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR})

install(EXPORT DBoW2Targets
  FILE DBoW2Targets.cmake
  NAMESPACE DBoW2::
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/DBoW2/)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake
  DESTINATION ${INSTALL_CMAKEDIR}/DBoW2)
