project(lzfse C)
cmake_minimum_required(VERSION 2.8.6)

include(CheckCCompilerFlag)

# If LZFSE is being bundled in another project, we don't want to
# install anything.  However, we want to let people override this, so
# we'll use the LZFSE_BUNDLE_MODE variable to let them do that; just
# set it to OFF in your project before you add_subdirectory(lzfse).
get_directory_property(LZFSE_PARENT_DIRECTORY PARENT_DIRECTORY)
if("${LZFSE_BUNDLE_MODE}" STREQUAL "")
  # Bundled mode hasn't been set one way or the other, set the default
  # depending on whether or not we are the top-level project.
  if(LZFSE_PARENT_DIRECTORY)
    set(LZFSE_BUNDLE_MODE ON)
  else()
    set(LZFSE_BUNDLE_MODE OFF)
  endif(LZFSE_PARENT_DIRECTORY)
endif()
mark_as_advanced(LZFSE_BUNDLE_MODE)

if(NOT LZFSE_BUNDLE_MODE)
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
else()
  set(BUILD_SHARED_LIBS OFF)
endif()

if (CMAKE_VERSION VERSION_GREATER 3.2)
  cmake_policy (SET CMP0063 NEW)
endif ()

# Compiler flags
function(lzfse_add_compiler_flags target)
  set (flags ${ARGV})
  list (REMOVE_AT flags 0)

  foreach (FLAG ${flags})
    if(CMAKE_C_COMPILER_ID STREQUAL GNU)
      # Because https://gcc.gnu.org/wiki/FAQ#wnowarning
      string(REGEX REPLACE "\\-Wno\\-(.+)" "-W\\1" flag_to_test "${FLAG}")
    else()
      set (flag_to_test ${FLAG})
    endif()

    string(REGEX REPLACE "[^a-zA-Z0-9]+" "_" test_name "CFLAG_${flag_to_test}")

    check_c_compiler_flag("${flag_to_test}" "${test_name}")
    if(${${test_name}})
      set_property(TARGET "${target}" APPEND_STRING PROPERTY COMPILE_FLAGS " ${FLAG}")
    endif()
  endforeach()
endfunction()

add_library(lzfse
  src/lzfse_decode.c
  src/lzfse_decode_base.c
  src/lzfse_encode.c
  src/lzfse_encode_base.c
  src/lzfse_fse.c
  src/lzvn_decode_base.c
  src/lzvn_encode_base.c)
lzfse_add_compiler_flags(lzfse -Wall -Wno-unknown-pragmas -Wno-unused-variable)

add_executable(lzfse_cli
  src/lzfse_main.c)
target_link_libraries(lzfse_cli lzfse)
set_target_properties(lzfse_cli PROPERTIES OUTPUT_NAME lzfse)
lzfse_add_compiler_flags(lzfse_cli -Wall -Wno-unknown-pragmas -Wno-unused-variable)

if(CMAKE_VERSION VERSION_LESS 3.1 OR CMAKE_C_COMPLIER_ID STREQUAL "Intel")
  lzfse_add_compiler_flags(lzfse -std=c99)
  lzfse_add_compiler_flags(lzfse_cli -std=c99)
else()
  set_property(TARGET lzfse PROPERTY C_STANDARD 99)
  set_property(TARGET lzfse_cli PROPERTY C_STANDARD 99)
endif()

set_target_properties(lzfse PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  C_VISIBILITY_PRESET hidden
  INTERPROCEDURAL_OPTIMIZATION TRUE)

# Installation
if(NOT LZFSE_BUNDLE_MODE)
  include(GNUInstallDirs)

  install(TARGETS lzfse lzfse_cli
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
  install(FILES src/lzfse.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()
