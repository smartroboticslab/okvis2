cmake_minimum_required(VERSION 3.8...3.16)

# Copy file SOURCE to DESTINATION if DESTINATION exists and show an error
# message otherwise.
function(copy_if_exists SOURCE DESTINATION)
  if(EXISTS "${DESTINATION}")
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SOURCE}" "${DESTINATION}")
  else()
    message(FATAL_ERROR "File ${DESTINATION} doesn't exist. Did you forget to update the git submodules? Run:\ngit submodule update --init --recursive\n")
  endif()
endfunction()

if(NOT USE_SYSTEM_BRISK)
  option(BRISK_BUILD_DEMO OFF)
  add_subdirectory(brisk)
endif()

if(NOT USE_SYSTEM_DBOW2)
  # Use a modified CMakeLists.txt for DBoW2 to allow building as a static
  # library and to fix issues when installing it along with okvis.
  copy_if_exists(${CMAKE_CURRENT_SOURCE_DIR}/patches/DBoW2/CMakeLists.txt ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/CMakeLists.txt)
  option(BUILD_Demo OFF)
  add_subdirectory(DBoW2)
  set(DBoW2_LIBRARIES DBoW2 PARENT_SCOPE)
endif()

if(NOT USE_SYSTEM_CERES)
  if(OPENMP_FOUND)
    set(CERES_THREADING_MODEL OPENMP)
  else()
    set(CERES_THREADING_MODEL CXX_THREADS)
  endif()
  option(BUILD_TESTING OFF)
  option(BUILD_EXAMPLES OFF)
  option(BUILD_BENCHMARKS OFF)
  add_subdirectory(ceres-solver)
endif()

# Use a modified CMakeLists.txt for OpenGV to build with the same flags as
# okvis2.
copy_if_exists(${CMAKE_CURRENT_SOURCE_DIR}/patches/opengv/CMakeLists.txt ${CMAKE_CURRENT_SOURCE_DIR}/opengv/CMakeLists.txt)
add_subdirectory(opengv)
set(OPENGV_LIBRARIES opengv PARENT_SCOPE)

if(BUILD_TESTS)
  option(INSTALL_GTEST OFF)
  add_subdirectory(googletest)
endif()
