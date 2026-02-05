# ============================================================
# CORA examples
# ============================================================
project(cora-main)
message(STATUS "Building CORA examples")

# ------------------------------------------------------------
# Main command-line example
# ------------------------------------------------------------




if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O1 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")






add_executable(cora_example lab1.cpp)




target_include_directories(cora_example PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)



target_include_directories(cora_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/Optimization/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    /usr/include/eigen3
    /usr/include/suitesparse
)

target_link_directories(cora_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/lib
)

target_link_libraries(cora_example PRIVATE
    CORA
)

    
# Profiling (optional)
if(ENABLE_PROFILING)
    target_compile_definitions(cora_example PRIVATE GPERFTOOLS)
    target_link_libraries(cora_example profiler)
endif()



# ------------------------------------------------------------
# Visualization example (optional)
# ------------------------------------------------------------
if(ENABLE_VISUALIZATION)
    message(STATUS "Building CORAVis example")

    add_executable(cora_vis cora_vis_test.cpp)

    target_include_directories(cora_vis PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )

    target_link_libraries(cora_vis
        CORA
    )
endif()

# ------------------------------------------------------------
# Paper experiments (optional)
# ------------------------------------------------------------
if(PERFORM_EXPERIMENTS)
    message(STATUS "Building paper_experiments executable")

    add_executable(paper_experiments paper_experiments.cpp)

target_link_directories(paper_experiments PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/lib
)



    target_include_directories(paper_experiments PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/Optimization/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    /usr/include/eigen3
    /usr/include/suitesparse
)


    target_link_libraries(paper_experiments PRIVATE
        CORA
    )

    if(ENABLE_PROFILING)
        target_compile_definitions(paper_experiments PRIVATE GPERFTOOLS)
        target_link_libraries(paper_experiments profiler )
    endif()
endif()

# ------------------------------------------------------------
# Copy example PyFG files to build directory
# ------------------------------------------------------------
set(EXAMPLE_PYFG_FILES
    config.json
    data/factor_graph_small.pyfg
    data/mrclam/range_and_rpm/mrclam2/mrclam2.pyfg
    data/mrclam/range_and_rpm/mrclam3a/mrclam3a.pyfg
    data/mrclam/range_and_rpm/mrclam3b/mrclam3b.pyfg
    data/mrclam/range_and_rpm/mrclam4/mrclam4.pyfg
    data/mrclam/range_and_rpm/mrclam5a/mrclam5a.pyfg
    data/mrclam/range_and_rpm/mrclam5b/mrclam5b.pyfg
    data/mrclam/range_and_rpm/mrclam5c/mrclam5c.pyfg
    data/mrclam/range_and_rpm/mrclam6/mrclam6.pyfg
    data/mrclam/range_and_rpm/mrclam7/mrclam7.pyfg
    data/plaza1.pyfg
    data/plaza2.pyfg
    data/single_drone.pyfg
    data/tiers.pyfg
)

message(STATUS "Copying example PyFG files to ${EXECUTABLE_OUTPUT_PATH}")

foreach(PYFG_FILE ${EXAMPLE_PYFG_FILES})
    configure_file(
        ${PYFG_FILE}
        ${EXECUTABLE_OUTPUT_PATH}/${PYFG_FILE}
        COPYONLY
    )
endforeach()
