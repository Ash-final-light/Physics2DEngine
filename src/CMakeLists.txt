add_library(box2d
	pogo_joint.cpp
	mover_joint.cpp
	motor_joint.cpp
	weld_joint.cpp
	wheel_joint.cpp
	prismatic_joint.cpp
	revolute_joint.cpp
	distance_joint.cpp
	joint.cpp
	mover.cpp
	sensor.cpp
	contact_solver.cpp
	contact_solver_w4.cpp
	contact_solver_w8.cpp
	constraint_graph.cpp
	island.cpp
	contact.cpp
	core.cpp
	math_functions.cpp
	aabb.cpp
	geometry.cpp
	distance.cpp
	manifold.cpp
	chain_builder.cpp
	dynamic_tree.cpp
	hull.cpp
	bitset.cpp
	id_pool.cpp
	table.cpp
	arena_allocator.cpp
	timer.cpp
	types.cpp
	physics_world.cpp
	broad_phase.cpp
	body.cpp
	shape.cpp
	solver.cpp
	solver_set.cpp
	parallel_for.cpp
	scheduler.cpp
	simd.cpp
	state_hash.cpp
	snapshot.cpp
	recording.cpp
	replay.cpp
)
add_library(box2d::box2d ALIAS box2d)

target_include_directories(box2d PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_compile_features(box2d PUBLIC cxx_std_20)

if(MSVC AND NOT BOX2D_DISABLE_SIMD AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_ARCHITECTURE_ID MATCHES "^(x64|X86)$")
 set_source_files_properties(contact_solver_w8.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2")
elseif(MINGW AND NOT BOX2D_DISABLE_SIMD AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
 set_source_files_properties(contact_solver_w8.cpp PROPERTIES COMPILE_OPTIONS "-Wa,-muse-unaligned-vector-move")
endif()

set_target_properties(box2d PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)
if(WIN32 AND BUILD_SHARED_LIBS)
 target_compile_definitions(box2d INTERFACE BOX2D_DLL)
endif()

# C consumers of the static C++ implementation need the C++ runtime.
set(BOX2D_CXX_RUNTIME "")
if(APPLE)
 set(BOX2D_CXX_RUNTIME "c++")
elseif((UNIX OR MINGW) AND NOT MSVC)
 set(BOX2D_CXX_RUNTIME "stdc++")
endif()
if(NOT BUILD_SHARED_LIBS AND BOX2D_CXX_RUNTIME)
 target_link_libraries(box2d INTERFACE ${BOX2D_CXX_RUNTIME})
endif()
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
 target_link_libraries(box2d PUBLIC m)
endif()
