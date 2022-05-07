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

option(BUILD_PACKAGING "Enable build of distribution packages using CPack." OFF)
option(BUILD_PACKAGING_SOURCES "Enable build package sources using CPack." OFF)
option(INSTALL_HEADERS "Request packaging of headers and other development files." OFF)
mark_as_advanced(INSTALL_HEADERS)

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

if (ENABLE_SANITIZER)
  set(CMAKE_C_FLAGS " ${CMAKE_C_FLAGS} -fsanitize=${ENABLE_SANITIZER}")
  set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -fsanitize=${ENABLE_SANITIZER}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=${ENABLE_SANITIZER}")
endif ()

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

if(BUILD_SHARED_LIBS)
  set_property(TARGET lzfse APPEND PROPERTY COMPILE_DEFINITIONS LZFSE_DLL LZFSE_DLL_EXPORTS)
  set_property(TARGET lzfse APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS LZFSE_DLL)
endif()

# Installation
if(NOT LZFSE_BUNDLE_MODE)
  include(GNUInstallDirs)

  if(BUILD_PACKAGING AND INSTALL_HEADERS)
    install(FILES src/lzfse.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
  elseif(BUILD_PACKAGING)
    install(TARGETS lzfse lzfse_cli
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
  else()
    install(FILES src/lzfse.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    install(TARGETS lzfse lzfse_cli
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
  endif()
endif()

# Tests

# If we're targeting Windows but not running on Windows, we need Wine
# to run the tests...
if(NOT LZFSE_DISABLE_TESTS)
  if(WIN32 AND NOT CMAKE_HOST_WIN32)
    find_program(LZFSE_WINE NAMES wine)

    if(NOT LZFSE_WINE)
      message(STATUS "wine not found, disabling tests")
      set(LZFSE_DISABLE_TESTS TRUE)
    endif()
  endif()
endif()

if(NOT LZFSE_DISABLE_TESTS)
  include(CTest)
  enable_testing()

  file(GLOB_RECURSE
    ROUNDTRIP_INPUTS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*)

  foreach(INPUT ${ROUNDTRIP_INPUTS})
    add_test(NAME "${LZFSE_TEST_PREFIX}roundtrip/${INPUT}"
      COMMAND "${CMAKE_COMMAND}"
        -DLZFSE_WRAPPER=${LZFSE_WINE}
        -DLZFSE_CLI=$<TARGET_FILE:lzfse_cli>
        -DINPUT=${CMAKE_CURRENT_SOURCE_DIR}/${INPUT}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/round-trip.cmake)
  endforeach()
endif()

# Packaging
if(BUILD_PACKAGING)

  if(BUILD_PACKAGING_SOURCES)
    set(SRPM TRUE)
  endif()

  set(PACKAGE_NAME "lzfse")
  set(PACKAGE_VERSION "1.0")
  set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
  set(PACKAGE_TARNAME "${PACKAGE_NAME}-${PACKAGE_VERSION}")

  if(NOT BUILD_SHARED_LIBS AND NOT INSTALL_HEADERS)
    message(WARNING "Package will contain static libraries without headers!"
                    "\nRecommended options for generation of runtime package:"
                    "\n  BUILD_SHARED_LIBS=ON"
                    "\n  BUILD_STATIC_LIBS=OFF"
                    "\n  INSTALL_HEADERS=OFF"
                    "\nRecommended options for generation of development package:"
                    "\n  BUILD_SHARED_LIBS=ON"
                    "\n  BUILD_STATIC_LIBS=ON"
                    "\n  INSTALL_HEADERS=ON")
  endif()

  find_program(DPKG dpkg)
  find_program(RPM rpm)

  # default package generators
  if(APPLE)
    set(PACKAGE_GENERATOR "PackageMaker")
    set(PACKAGE_SOURCE_GENERATOR "TGZ;ZIP")
  elseif(UNIX)
    if(DPKG AND NOT RPM)
      set(PACKAGE_GENERATOR "DEB")
    elseif(RPM AND NOT DPKG)
      set(PACKAGE_GENERATOR "RPM")
    endif()
    set(PACKAGE_SOURCE_GENERATOR "TGZ;ZIP")
  else()
    set(PACKAGE_GENERATOR "ZIP")
    set(PACKAGE_SOURCE_GENERATOR "ZIP")
  endif()

  # used package generators
  set(CPACK_GENERATOR "${PACKAGE_GENERATOR}" CACHE STRING "List of binary package generators (CPack).")
  set(CPACK_SOURCE_GENERATOR "${PACKAGE_SOURCE_GENERATOR}" CACHE STRING "List of source package generators (CPack).")
  mark_as_advanced(CPACK_GENERATOR CPACK_SOURCE_GENERATOR)

  # common package information
  set(CPACK_PACKAGE_VENDOR "Apple Inc.")
  set(CPACK_PACKAGE_NAME "${PACKAGE_NAME}")
  set(CPACK_PACKAGE_VERSION "${PACKAGE_VERSION}")
  set(CPACK_PACKAGE_VERSION_MAJOR "${PACKAGE_VERSION_MAJOR}")
  set(CPACK_PACKAGE_VERSION_MINOR "${PACKAGE_VERSION_MINOR}")
  set(CPACK_PACKAGE_VERSION_PATCH "${PACKAGE_VERSION_PATCH}")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "LZFSE compression library and command line tool.")
  set(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_CURRENT_LIST_DIR}/README.md")
  set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
  set(CPACK_OUTPUT_FILE_PREFIX packages)
  set(CPACK_PACKAGE_RELOCATABLE TRUE)
  set(CPACK_MONOLITHIC_INSTALL TRUE)

  # RPM package information -- used in cmake/package.cmake.in also for DEB
  set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
  set(CPACK_RPM_PACKAGE_LICENSE "BSD")
  set(CPACK_RPM_PACKAGE_URL "https://github.com/lzfse/lzfse")
  set(CPACK_RPM_PACKAGE_REQUIRES "cmake, gcc")
  set(CPACK_RPM_CHANGELOG_FILE "${CMAKE_CURRENT_LIST_DIR}/ChangeLog.txt")

  # system/architecture
  if(WINDOWS)
    if(CMAKE_CL_64)
      set(CPACK_SYSTEM_NAME "win64")
    else()
      set(CPACK_SYSTEM_NAME "win32")
    endif()
    set(CPACK_PACKAGE_ARCHITECTURE)
  elseif(APPLE)
    set(CPACK_PACKAGE_ARCHITECTURE darwin)
  else()
    string(TOLOWER "${CMAKE_SYSTEM_NAME}" CPACK_SYSTEM_NAME)
    if(CMAKE_CXX_FLAGS MATCHES "-m32")
      set(CPACK_PACKAGE_ARCHITECTURE i386)
    else()
      if(DPKG AND NOT RPM)
        execute_process(
          COMMAND dpkg --print-architecture
          RESULT_VARIABLE RV
          OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE
        )
      else()
        execute_process(
          COMMAND rpm --eval "%{_arch}"
          RESULT_VARIABLE RV
          OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE
        )
      endif()

      if(RV EQUAL 0)
	      string(STRIP "${CPACK_PACKAGE_ARCHITECTURE}" CPACK_PACKAGE_ARCHITECTURE)
      else()
        execute_process(COMMAND uname -m OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE)
        if(CPACK_PACKAGE_ARCHITECTURE MATCHES "x86_64")
	        set(CPACK_PACKAGE_ARCHITECTURE amd64)
        else()
          set(CPACK_PACKAGE_ARCHITECTURE i386)
        endif()
      endif()
    endif()
  endif()

  # source package settings
  set(CPACK_SOURCE_TOPLEVEL_TAG "source")
  set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
  set(CPACK_SOURCE_IGNORE_FILES "/\\\\.git/;\\\\.swp$;\\\\.#;/#;\\\\.*~;cscope\\\\.*;/[Bb]uild[.+-_a-zA-Z0-9]*/")

  # default binary package settings
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY TRUE)
  set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
  if(CPACK_PACKAGE_ARCHITECTURE)
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}-${CPACK_PACKAGE_ARCHITECTURE}")
  endif()

  # generator specific configuration file
  #
  # allow package maintainers to use their own configuration file
  # $ cmake -DCPACK_PROJECT_CONFIG_FILE:FILE=/path/to/package/config
  if(NOT CPACK_PROJECT_CONFIG_FILE)
    configure_file(
	    "${CMAKE_CURRENT_LIST_DIR}/modules/Package.cmake.in"
      "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}-package.cmake" @ONLY
    )
    set(CPACK_PROJECT_CONFIG_FILE "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}-package.cmake")
  endif()

  include(CPack)

endif()
