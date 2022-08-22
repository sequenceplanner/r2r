#
# For r2r 0.6.3.
#
# cmake code for simple colcon integration.
# See https://github.com/m-dahl/r2r_minimal_node/
# for an example of how to use it to build with colcon.
#

# the "recursive" dependencies already defined don't seem to include all
# packages. sigh. so here we traverse the dependencies manually
# instead. we also keep track of which packages that contain idl files.
function(get_idl_deps OUT_INC OUT_LIB OUT_PKG_DIRS PKG)
    find_package(${PKG} REQUIRED)
    list(APPEND VISITED_TARGETS ${PKG})

    set(INCS "${${PKG}_INCLUDE_DIRS}")
    set(LIBS "${${PKG}_LIBRARIES}")

    # only keep track of packages that include idl files
    set(PKG_DIRS "")
    if(DEFINED ${PKG}_IDL_FILES)
        list(APPEND PKG_DIRS ${${PKG}_DIR})
    endif()

    foreach(LIB ${${PKG}_DEPENDENCIES})
        list(FIND VISITED_TARGETS ${LIB} VISITED)
        if (${VISITED} EQUAL -1)
            get_idl_deps(NEW_INCS NEW_LIBS NEW_PKG_DIRS ${LIB})
            list(APPEND INCS ${NEW_INCS})
            list(APPEND LIBS ${NEW_LIBS})
            list(APPEND PKG_DIRS ${NEW_PKG_DIRS})
            list(REMOVE_DUPLICATES INCS)
            list(REMOVE_DUPLICATES LIBS)
            list(REMOVE_DUPLICATES PKG_DIRS)
        endif()
    endforeach()
    set(VISITED_TARGETS ${VISITED_TARGETS} PARENT_SCOPE)
    set(${OUT_INC} ${INCS} PARENT_SCOPE)
    set(${OUT_LIB} ${LIBS} PARENT_SCOPE)
    set(${OUT_PKG_DIRS} ${PKG_DIRS} PARENT_SCOPE)
endfunction()

function(r2r_cargo)
  foreach(f ${ARGN})
    find_package(${f} REQUIRED)
    set(REC_INC "")
    set(REC_LIB "")
    set(REC_PKG_DIRS "")
    get_idl_deps(REC_INC REC_LIB REC_PKG_DIRS ${f})
    list(APPEND CMAKE_INCLUDE_DIRS ${REC_INC})
    list(APPEND CMAKE_LIBRARIES ${REC_LIB})
    list(APPEND CMAKE_IDL_PACKAGES "${REC_PKG_DIRS}")
  endforeach()
  list(REMOVE_DUPLICATES CMAKE_INCLUDE_DIRS)
  string (REPLACE ";" ":" CMAKE_INCLUDE_DIRS_STR "${CMAKE_INCLUDE_DIRS}")
  set(ENV{CMAKE_INCLUDE_DIRS} ${CMAKE_INCLUDE_DIRS_STR})
  list(REMOVE_DUPLICATES CMAKE_LIBRARIES)

  # On OSX  colcon eats the DYLD_LIBRARY_PATH... so we need to add the rpaths
  # manually...
  set(RUSTFLAGS "")
  foreach(p ${CMAKE_LIBRARIES})
    get_filename_component(_parent "${p}" DIRECTORY)
    if(IS_DIRECTORY ${_parent})
        list(APPEND RUSTFLAGS "-C link-arg=-Wl,-rpath,${_parent}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES RUSTFLAGS)

  string (REPLACE ";" " " RUSTFLAGS_STR "${RUSTFLAGS}")
  set(ENV{RUSTFLAGS} ${RUSTFLAGS_STR})

  string (REPLACE ":" "+" CMAKE_LIBRARIES_STR "${CMAKE_LIBRARIES}")
  string (REPLACE ";" ":" CMAKE_LIBRARIES_STR "${CMAKE_LIBRARIES_STR}")
  set(ENV{CMAKE_LIBRARIES} "${CMAKE_LIBRARIES_STR}")

  list(REMOVE_DUPLICATES CMAKE_IDL_PACKAGES)
  string (REPLACE ";" ":" CMAKE_IDL_PACKAGES_STR "${CMAKE_IDL_PACKAGES}")
  set(ENV{CMAKE_IDL_PACKAGES} ${CMAKE_IDL_PACKAGES_STR})

  # custom target for building using cargo
  option(CARGO_CLEAN "Invoke cargo clean before building" OFF)
  if(CARGO_CLEAN)
        add_custom_target(cargo_target ALL
              COMMAND ${CMAKE_COMMAND} "-E" "env" "cargo" "clean"
              COMMAND ${CMAKE_COMMAND} "-E" "env" "RUSTFLAGS=$ENV{RUSTFLAGS}" "CMAKE_INCLUDE_DIRS=$ENV{CMAKE_INCLUDE_DIRS}" "CMAKE_LIBRARIES=$ENV{CMAKE_LIBRARIES}" "CMAKE_IDL_PACKAGES=$ENV{CMAKE_IDL_PACKAGES}" "cargo" "build" "--release"
              WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
              )
  else()
          add_custom_target(cargo_target ALL
              COMMAND ${CMAKE_COMMAND} "-E" "env" "RUSTFLAGS=$ENV{RUSTFLAGS}" "CMAKE_INCLUDE_DIRS=$ENV{CMAKE_INCLUDE_DIRS}" "CMAKE_LIBRARIES=$ENV{CMAKE_LIBRARIES}" "CMAKE_IDL_PACKAGES=$ENV{CMAKE_IDL_PACKAGES}" "cargo" "build" "--release"
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
              )
  endif(CARGO_CLEAN)
  unset(CARGO_CLEAN CACHE)

endfunction()
