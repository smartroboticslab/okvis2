cmake_minimum_required(VERSION 3.8...3.30)

find_package(Eigen3 REQUIRED)

set(LIB_NAME okvis_kinematics)
add_library(${LIB_NAME} INTERFACE
  #include/okvis/kinematics/operators.hpp
  #include/okvis/kinematics/Transformation.hpp
  #include/okvis/kinematics/implementation/Transformation.hpp
)
target_include_directories(okvis_util
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_NAME}
  INTERFACE
    Eigen3::Eigen
    okvis::Util
)
target_compile_features(${LIB_NAME} INTERFACE cxx_std_${OKVIS_CXX_STANDARD})
add_library(okvis::Kinematics ALIAS ${LIB_NAME})

install(TARGETS ${LIB_NAME} EXPORT okvisTargets)
install(DIRECTORY include/okvis DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp")

if(BUILD_TESTS)
  set(TEST_NAME okvis_kinematics_test)
  add_executable(${TEST_NAME}
    test/TestTransformation.cpp
    test/runTests.cpp
  )
  target_link_libraries(${TEST_NAME}
    ${LIB_NAME}
    gtest
  )
  target_compile_options(${TEST_NAME}
    PUBLIC
      ${OKVIS_PUBLIC_CXX_FLAGS}
    PRIVATE
      ${OKVIS_PRIVATE_CXX_FLAGS}
  )
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endif()
