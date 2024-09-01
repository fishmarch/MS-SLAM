# ------------------------------------------------------------------------------
# This file sets up Gurobi for CMake. Once done this will define
#
#   GUROBI_FOUND           - system has GUROBI
#   GUROBI_INCLUDE_DIRS    - the GUROBI include directories
#   GUROBI_LIBRARIES       - Link these to use GUROBI
#
#  In your CMakeLists file, you need to add, e.g. (modify it if necessary):
#        if (GUROBI_FOUND)
#            message(STATUS "Gurobi include dir: " ${GUROBI_INCLUDE_DIRS})
#            message(STATUS "Gurobi libraries: " ${GUROBI_LIBRARIES})
#            target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_GUROBI)
#            target_include_directories(${PROJECT_NAME} PRIVATE ${GUROBI_INCLUDE_DIRS})
#            target_link_libraries(${PROJECT_NAME} PRIVATE ${GUROBI_LIBRARIES})
#        endif()
# ------------------------------------------------------------------------------


# Is it already configured?
if (NOT GUROBI_FOUND)

    # Hardcoded search paths
    set(SEARCH_PATHS_FOR_HEADERS
            "$ENV{GUROBI_HOME}/include"
            "/home/zxy/myLibs/gurobi1002/linux64/include"
            )

    set(SEARCH_PATHS_FOR_LIBRARIES
            "$ENV{GUROBI_HOME}/lib"
            "/home/zxy/myLibs/gurobi1002/linux64/lib"
            )

    find_path(GUROBI_INCLUDE_DIR gurobi_c++.h
            PATHS ${SEARCH_PATHS_FOR_HEADERS}
            )


    find_library( GUROBI_C_LIBRARY
            NAMES gurobi100 libgurobi100 libgurobi
            PATHS ${SEARCH_PATHS_FOR_LIBRARIES}
            )

    find_library( GUROBI_CXX_LIBRARY_DEBUG
            NAMES gurobi_c++ gurobi_c++mdd2017
            PATHS ${SEARCH_PATHS_FOR_LIBRARIES}
            )

    find_library( GUROBI_CXX_LIBRARY_RELEASE
            NAMES gurobi_c++ gurobi_c++md2017
            PATHS ${SEARCH_PATHS_FOR_LIBRARIES}
            )

    # setup header file directories
    set(GUROBI_INCLUDE_DIRS ${GUROBI_INCLUDE_DIR})

    # setup libraries files
    set(GUROBI_LIBRARIES
            debug ${GUROBI_CXX_LIBRARY_DEBUG}
            optimized ${GUROBI_CXX_LIBRARY_RELEASE}
            ${GUROBI_C_LIBRARY}
            )

endif()

# Check that Gurobi was successfully found
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_INCLUDE_DIRS)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARIES)

# Hide variables from CMake-Gui options
mark_as_advanced(GUROBI_LIBRARIES GUROBI_INCLUDE_DIRS GUROBI_INCLUDE_DIR)