cmake_minimum_required(VERSION 3.8...3.30)

# Don't use find_package(Boost REQUIRED COMPONENTS headers) or link using the
# newer Boost::headers target to retain compatibility with the FindBoost.cmake
# shipped in Ubuntu 18.04.
find_package(Boost REQUIRED)

set(LIB_NAME okvis_timing)
add_library(${LIB_NAME}
  include/okvis/timing/NsecTimeUtilities.hpp
  include/okvis/timing/Timer.hpp
  src/NsecTimeUtilities.cpp
  src/Timer.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  Boost::boost
  okvis::Time
  okvis::Util
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${OKVIS_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${OKVIS_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${OKVIS_PRIVATE_CXX_FLAGS}
)
add_library(okvis::Timing ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT okvisTargets 
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/okvis DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
