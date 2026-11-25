add_library(box2d
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
)
add_library(box2d::box2d ALIAS box2d)

target_include_directories(box2d PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_compile_features(box2d PUBLIC cxx_std_20)
