# Particle liquids for Box2D v3: the library, and the lab tools that run its scenes without a window

set(FLUIDFUN_FILES
	include/fluidfun/fluid.h
	src/internal.h
	src/fluid.c
	src/grid.c
	src/solver.c
	src/boundary.c
	src/adapt.c
	src/spawn.c
)

add_library(fluidfun STATIC ${FLUIDFUN_FILES})
target_include_directories(fluidfun PUBLIC include)
target_link_libraries(fluidfun PUBLIC box2d)
set_target_properties(fluidfun PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${FLUIDFUN_FILES})

if(NOT PROJECT_IS_TOP_LEVEL)
	return()
endif()

# Scenes shared by the samples and the headless runner
add_library(fluid_scenes STATIC lab/scene.c lab/scene.h)
target_include_directories(fluid_scenes PUBLIC lab)
target_link_libraries(fluid_scenes PUBLIC fluidfun PRIVATE m)
set_target_properties(fluid_scenes PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

add_executable(fluid_headless lab/headless.c)
target_link_libraries(fluid_headless PRIVATE fluid_scenes m)
set_target_properties(fluid_headless PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

add_executable(fluid_compare lab/compare.c)
target_link_libraries(fluid_compare PRIVATE fluidfun m)
set_target_properties(fluid_compare PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

# Every scene runs at 1, 3, 6 and 12 workers: the final state must hash the same, and no liquid may leak.
# Testing is enabled here only, since Box2D's own unit test target is named test, which CTest reserves.
# Run with: ctest --test-dir build/fluid
enable_testing()
set(FLUID_SCENES tank hole boat drum wheel waves splash drain pool)
list(LENGTH FLUID_SCENES FLUID_SCENE_COUNT)
math(EXPR FLUID_LAST_SCENE "${FLUID_SCENE_COUNT} - 1")
foreach(index RANGE ${FLUID_LAST_SCENE})
	list(GET FLUID_SCENES ${index} name)
	add_test(NAME fluid_${name} COMMAND fluid_headless 300)
	set_tests_properties(fluid_${name} PROPERTIES ENVIRONMENT "LAB_SCENE=${index};LAB_CHECK=1")
endforeach()

# The same with uniform sizes, which take their own paths through the solver
foreach(index 0 2 4)
	list(GET FLUID_SCENES ${index} name)
	add_test(NAME fluid_uniform_${name} COMMAND fluid_headless 300)
	set_tests_properties(fluid_uniform_${name} PROPERTIES ENVIRONMENT "LAB_SCENE=${index};LAB_ADAPTIVE=1;LAB_CHECK=1")
endforeach()
