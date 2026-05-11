file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(planner_fsm STATIC ${srcs})
target_include_directories(planner_fsm PUBLIC include)
target_link_libraries(planner_fsm PUBLIC
        super_planner
        rog_map
        mars_planning_utils
        ${Boost_LIBRARIES})