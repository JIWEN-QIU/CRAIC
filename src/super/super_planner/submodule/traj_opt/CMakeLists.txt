#message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
#if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
#    include(ProcessorCount)
#    ProcessorCount(N)
#    message("Processer number:  ${N}")
#    if (N GREATER 4)
#        add_definitions(-DMP_EN)
#        add_definitions(-DMP_PROC_NUM=3)
#        message("core for MP: 3")
#    elseif (N GREATER 3)
#        add_definitions(-DMP_EN)
#        add_definitions(-DMP_PROC_NUM=2)
#        message("core for MP: 2")
#    else ()
#        add_definitions(-DMP_PROC_NUM=1)
#    endif ()
#else ()
#    add_definitions(-DMP_PROC_NUM=1)
#endif ()
#find_package(OpenMP REQUIRED)
##include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})
##set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(traj_opt STATIC ${srcs})
target_include_directories(traj_opt PUBLIC
        include
        ${Boost_LIBRARIES})
target_link_libraries(traj_opt PUBLIC
        mars_planning_utils
        ${Boost_LIBRARIES}
#        OpenMP::OpenMP_CXX
        )

