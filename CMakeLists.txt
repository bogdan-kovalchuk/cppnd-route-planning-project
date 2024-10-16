cmake_minimum_required(VERSION 3.11.3)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# Set the C++ standard we will use
set(CMAKE_CXX_STANDARD 17)

# Add the path of the cmake files to the CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

project(OSM_A_star_search)

option(BUILD_APP "Build the interactive route planner" ON)

# Set library output path to /lib
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/lib")

if(MSVC)
    set(gtest_force_shared_crt ON CACHE BOOL "Use the shared runtime for GoogleTest" FORCE)
endif()

# Add the pugixml and GoogleTest library subdirectories
add_subdirectory(thirdparty/pugixml)
add_subdirectory(thirdparty/googletest)

if(BUILD_APP)
    # Locate prerequisites needed only by the interactive renderer.
    find_package(io2d REQUIRED)
    find_package(Cairo)
    find_package(GraphicsMagick)

    # Set IO2D flags
    set(IO2D_WITHOUT_SAMPLES 1)
    set(IO2D_WITHOUT_TESTS 1)

    add_executable(OSM_A_star_search src/main.cpp src/model.cpp src/render.cpp src/route_model.cpp src/route_planner.cpp)

    target_link_libraries(OSM_A_star_search
        PRIVATE io2d::io2d
        PUBLIC pugixml
    )

    if( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
        target_link_libraries(OSM_A_star_search PUBLIC pthread)
    endif()

    if(MSVC)
        target_compile_options(OSM_A_star_search PUBLIC /D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING /wd4459)
    endif()
endif()

# Add the testing executable
add_executable(route_planner_tests test/utest_rp_a_star_search.cpp src/route_planner.cpp src/model.cpp src/route_model.cpp)

target_link_libraries(route_planner_tests
    gtest_main 
    pugixml
)

enable_testing()
add_test(NAME RoutePlannerTests COMMAND route_planner_tests)
set_tests_properties(RoutePlannerTests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/test"
)
