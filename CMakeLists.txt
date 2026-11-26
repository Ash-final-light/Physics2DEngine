cmake_minimum_required(VERSION 3.22)

project(box2d
	VERSION 3.2.0
	DESCRIPTION "A 2D physics engine for games"
	HOMEPAGE_URL "https://box2d.org"
	LANGUAGES C CXX
)

# 源文件为 UTF-8（无 BOM）。MSVC 默认按系统代码页解析，中文 Windows 上会当成 GBK。
if(MSVC)
	add_compile_options(/utf-8)
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(PROJECT_IS_TOP_LEVEL)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
	include(CTest)
endif()

add_subdirectory(src)

add_executable(toolchain_check
	test/toolchain_c.c
	test/toolchain_cpp.cpp
)

if(BUILD_TESTING)
	add_executable(unit_tests
		test/main.cpp
		test/test_toolchain.cpp
		test/test_version.c
		test/test_assert.c
		test/test_log.cpp
		test/test_length.cpp
		test/test_memory.cpp
		test/test_scalar.cpp
		test/test_vector.cpp
		test/test_rotation.cpp
		test/test_transform.cpp
		test/test_matrix.cpp
		test/test_trig.cpp
		test/test_array.cpp
		test/test_bitset.cpp
		test/test_id_pool.cpp
		test/test_table.cpp
		test/test_stack.cpp
		test/test_timer.cpp
		test/test_aabb.cpp
		test/test_geometry.cpp
		test/test_distance.cpp
		test/test_collision.cpp
		test/test_chain.cpp
		test/test_dynamic_tree.cpp
		test/test_world.cpp
		test/test_step.cpp
		test/test_joint.cpp
		test/test_mover.cpp
		test/test_query.cpp
		test/test_sensor.cpp
		test/test_contact.cpp
		test/test_solver_set.cpp
		test/test_island.cpp
		test/test_constraint_graph.cpp
		test/test_contact_solver.cpp
		test/test_shape.cpp
		test/test_broad_phase.cpp
		test/test_hull.cpp
		test/toolchain_c.c
	)
	target_include_directories(unit_tests PRIVATE
		"${CMAKE_CURRENT_SOURCE_DIR}/test"
		"${CMAKE_CURRENT_SOURCE_DIR}/src"
	)
	target_link_libraries(unit_tests PRIVATE box2d)
	add_test(NAME unit_tests COMMAND unit_tests)
endif()
