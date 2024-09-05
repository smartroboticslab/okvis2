cmake_minimum_required(VERSION 3.8...3.30)

set(LIB_NAME okvis_time)
add_library(${LIB_NAME}
  include/okvis/Duration.hpp
  include/okvis/Time.hpp
  include/okvis/implementation/Duration.hpp
  include/okvis/implementation/Time.hpp
  src/Duration.cpp
  src/Time.cpp
)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${LIB_NAME} PUBLIC cxx_std_${OKVIS_CXX_STANDARD})
target_compile_options(${LIB_NAME}
  PUBLIC
    ${OKVIS_PUBLIC_CXX_FLAGS}
  PRIVATE
    ${OKVIS_PRIVATE_CXX_FLAGS}
)
add_library(okvis::Time ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT okvisTargets 
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/okvis DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")
