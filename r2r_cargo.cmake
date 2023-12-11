#
# For r2r 0.8.2.
#
# cmake code for simple colcon integration.
# See https://github.com/m-dahl/r2r_minimal_node/
# for an example of how to use it to build with colcon.
#

# Need at least 3.21 for IMPORTED_TARGETS property that we rely on.
# NOTE! Ubuntu 20.04 comes 3.16.3 by default, so to use this you need
# to update cmake. This can be done by adding the kitware apt repo:
# https://apt.kitware.com
cmake_minimum_required(VERSION "3.21")

# traverse dependencies for all message packages.
function(get_idl_deps OUT_PKG_DIRS PKG)
    find_package(${PKG} REQUIRED)
    target_link_libraries(dummy ${${PKG}_LIBRARIES})
    include_directories(dummy ${${PKG}_INCLUDE_DIRS})

    list(APPEND VISITED_TARGETS ${PKG})

    # only keep track of packages that include idl files
    set(PKG_DIRS "")
    if(DEFINED ${PKG}_IDL_FILES)
        list(APPEND PKG_DIRS ${${PKG}_DIR})
    endif()

    foreach(LIB ${${PKG}_DEPENDENCIES})
        list(FIND VISITED_TARGETS ${LIB} VISITED)
        if (${VISITED} EQUAL -1)
            get_idl_deps(NEW_PKG_DIRS ${LIB})
            list(APPEND PKG_DIRS ${NEW_PKG_DIRS})
            list(REMOVE_DUPLICATES PKG_DIRS)
        endif()
    endforeach()
    set(VISITED_TARGETS ${VISITED_TARGETS} PARENT_SCOPE)
    set(${OUT_PKG_DIRS} ${PKG_DIRS} PARENT_SCOPE)
endfunction()

function(r2r_cargo)
  # pretend that we want to compile c code to get all library paths etc...
  add_executable (dummy EXCLUDE_FROM_ALL dummy.c)

  # traverse list of wanted packages to add dependencies
  foreach(f ${ARGN})
    find_package(${f} REQUIRED)
    set(REC_PKG_DIRS "")
    get_idl_deps(REC_PKG_DIRS ${f})
    list(APPEND CMAKE_IDL_PACKAGES "${REC_PKG_DIRS}")
  endforeach()

  list(REMOVE_DUPLICATES CMAKE_IDL_PACKAGES)
  string (REPLACE ";" ":" CMAKE_IDL_PACKAGES_STR "${CMAKE_IDL_PACKAGES}")
  set(ENV{CMAKE_IDL_PACKAGES} ${CMAKE_IDL_PACKAGES_STR})

  # On OSX colcon eats the DYLD_LIBRARY_PATH... so we need to add the rpaths
  # manually...
  set(RUSTFLAGS "")

  # get imported libs
  get_property(importTargets DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)

  foreach(p ${importTargets})
    get_property(_LIBLOC TARGET "${p}" PROPERTY LOCATION)
    if(DEFINED _LIBLOC)
      list(APPEND CMAKE_LIBRARIES "${_LIBLOC}")
      get_filename_component(_PARENT "${_LIBLOC}" DIRECTORY)
      if(IS_DIRECTORY ${_PARENT})
          list(APPEND RUSTFLAGS "-C link-arg=-Wl,-rpath,${_PARENT}")
      endif()
    endif()
  endforeach()

  list(REMOVE_DUPLICATES RUSTFLAGS)
  string (REPLACE ";" " " RUSTFLAGS_STR "${RUSTFLAGS}")
  set(ENV{RUSTFLAGS} ${RUSTFLAGS_STR})

  # get include paths
  get_property(includeDirs DIRECTORY "${CMAKE_SOURCE_DIR}" PROPERTY INCLUDE_DIRECTORIES)
  list(REMOVE_DUPLICATES includeDirs)
  string (REPLACE ";" ":" CMAKE_INCLUDE_DIRS_STR "${includeDirs}")
  set(ENV{CMAKE_INCLUDE_DIRS} ${CMAKE_INCLUDE_DIRS_STR})

  # custom target for building using cargo
  option(CARGO_CLEAN "Invoke cargo clean before building" OFF)
  if(CARGO_CLEAN)
        add_custom_target(cargo_target ALL
              COMMAND ${CMAKE_COMMAND} "-E" "env" "cargo" "clean" "--profile" "colcon"
              COMMAND ${CMAKE_COMMAND} "-E" "env" "RUSTFLAGS=$ENV{RUSTFLAGS}" "CMAKE_IDL_PACKAGES=$ENV{CMAKE_IDL_PACKAGES}" "CMAKE_INCLUDE_DIRS=$ENV{CMAKE_INCLUDE_DIRS}" "cargo" "build" "--profile" "colcon"
              WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
              )
  else()
          add_custom_target(cargo_target ALL
              COMMAND ${CMAKE_COMMAND} "-E" "env" "RUSTFLAGS=$ENV{RUSTFLAGS}" "CMAKE_IDL_PACKAGES=$ENV{CMAKE_IDL_PACKAGES}" "CMAKE_INCLUDE_DIRS=$ENV{CMAKE_INCLUDE_DIRS}" "cargo" "build" "--profile" "colcon"
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
              )
  endif()
  unset(CARGO_CLEAN CACHE)

endfunction()
