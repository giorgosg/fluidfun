# Liquid scenes, shared by the samples, the headless runner that tests them, and a comparison tool

add_library(fluid_scenes STATIC scene.c scene.h)
target_include_directories(fluid_scenes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fluid_scenes PUBLIC box2d)
set_target_properties(fluid_scenes PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

add_executable(fluid_headless headless.c)
target_link_libraries(fluid_headless PRIVATE fluid_scenes)
set_target_properties(fluid_headless PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

add_executable(fluid_compare compare.c)
target_link_libraries(fluid_compare PRIVATE box2d)
set_target_properties(fluid_compare PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES)

if (BOX2D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(fluid_scenes fluid_headless fluid_compare PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

# Every scene runs at 1, 3, 6 and 12 workers: the final state must hash the same, and no liquid may leak.
# Testing is enabled here only, since Box2D's own unit test target is named test, which CTest reserves.
# Run with: ctest --test-dir build/benchmark/fluid
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
