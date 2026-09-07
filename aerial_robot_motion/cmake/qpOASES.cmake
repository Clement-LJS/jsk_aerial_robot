# Prefer an existing dependency prefix. Otherwise build a verified release
# archive entirely under the CMake binary directory; never install to the system.
find_path(QPOASES_INCLUDE_DIR qpOASES.hpp PATH_SUFFIXES qpOASES)
find_library(QPOASES_LIBRARY NAMES qpOASES)
if(QPOASES_INCLUDE_DIR AND QPOASES_LIBRARY)
  add_library(motion_qpoases UNKNOWN IMPORTED)
  set_target_properties(motion_qpoases PROPERTIES IMPORTED_LOCATION "${QPOASES_LIBRARY}")
else()
  include(ExternalProject)
  set(qpoases_source "${CMAKE_CURRENT_BINARY_DIR}/qpOASES-src")
  set(qpoases_binary "${CMAKE_CURRENT_BINARY_DIR}/qpOASES-build")
  set(QPOASES_INCLUDE_DIR "${qpoases_source}/include")
  ExternalProject_Add(motion_qpoases_build
    URL https://codeload.github.com/coin-or/qpOASES/tar.gz/31b5aa4fe80ba1dd1c13d569c232456c0b5c9791
    URL_HASH SHA256=d1d2a7afa7c7e8537276096d2f5f3fb32c409ed02efde485fc59700ef512c046
    SOURCE_DIR "${qpoases_source}"
    BINARY_DIR "${qpoases_binary}"
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
      -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target qpOASES
    BUILD_BYPRODUCTS "${qpoases_binary}/libs/libqpOASES.a"
    INSTALL_COMMAND "")
  add_library(motion_qpoases STATIC IMPORTED)
  set_target_properties(motion_qpoases PROPERTIES
    IMPORTED_LOCATION "${qpoases_binary}/libs/libqpOASES.a")
  add_dependencies(motion_qpoases motion_qpoases_build)
endif()
