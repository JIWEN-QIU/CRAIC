file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super_core STATIC ${srcs})
target_include_directories(super_core PUBLIC include)
target_link_libraries(super_core PUBLIC
        rog_map
        traj_opt
        mars_planning_utils
        ${Boost_LIBRARIES})