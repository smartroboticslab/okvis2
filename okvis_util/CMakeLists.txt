cmake_minimum_required(VERSION 3.8...3.30)

set(LIB_NAME okvis_util)
add_library(${LIB_NAME} INTERFACE
  #include/okvis/assert_macros.hpp
  #include/okvis/MstGraph.hpp
  #include/okvis/source_file_pos.hpp
)
target_include_directories(${LIB_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${LIB_NAME} INTERFACE cxx_std_${OKVIS_CXX_STANDARD})
add_library(okvis::Util ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT okvisTargets)
install(DIRECTORY include/okvis DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
