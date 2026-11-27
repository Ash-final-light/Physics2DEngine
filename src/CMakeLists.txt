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

target_include_directories(box2d PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
 $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(box2d PRIVATE cxx_std_20)

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

if(BOX2D_DOUBLE_PRECISION)
 target_compile_definitions(box2d PUBLIC BOX2D_DOUBLE_PRECISION)
endif()

set_target_properties(box2d PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
install(TARGETS box2d EXPORT box2dConfig
 LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
 ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../include/box2d" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT box2dConfig NAMESPACE box2d:: DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d")
export(EXPORT box2dConfig NAMESPACE box2d:: FILE "${PROJECT_BINARY_DIR}/box2dConfig.cmake")
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_BINARY_DIR}/box2dConfigVersion.cmake"
 VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES "${PROJECT_BINARY_DIR}/box2dConfigVersion.cmake" DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box2d")

# 所有路径相对 .pc 文件自身，安装树可通过 cmake --install --prefix
# 或事后搬迁仍能使用
file(RELATIVE_PATH PKGCONFIG_PREFIX_REL "${CMAKE_INSTALL_FULL_LIBDIR}/pkgconfig" "${CMAKE_INSTALL_PREFIX}")
string(REGEX REPLACE "/$" "" PKGCONFIG_PREFIX_REL "${PKGCONFIG_PREFIX_REL}")
file(RELATIVE_PATH PKGCONFIG_LIBDIR_REL "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_FULL_LIBDIR}")
file(RELATIVE_PATH PKGCONFIG_INCLUDEDIR_REL "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_FULL_INCLUDEDIR}")

# .pc 必须带上导出目标的全部 PUBLIC 宏，否则引用方按不同头文件语义编译。
# 需与上面的宏保持同步。
set(PKGCONFIG_CFLAGS "")
if (BOX2D_DOUBLE_PRECISION)
	string(APPEND PKGCONFIG_CFLAGS " -DBOX2D_DOUBLE_PRECISION")
endif()
if (WIN32 AND BUILD_SHARED_LIBS)
	# base.h 用 _WIN32 决定 dllimport，MinGW 和 MSVC 都需要此宏
	string(APPEND PKGCONFIG_CFLAGS " -DBOX2D_DLL")
endif()


set(PKGCONFIG_LIBNAME "box2d$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>")

# 普通 pkg-config --libs 会跳过 Libs.private；Box2D 默认静态库，
# 因此公共链接依赖应写在主行，除非正在构建共享库
set(PKGCONFIG_LIBS "")
set(PKGCONFIG_LIBS_PRIVATE "")
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
	if(BUILD_SHARED_LIBS)
		string(APPEND PKGCONFIG_LIBS_PRIVATE " -lm")
	else()
		string(APPEND PKGCONFIG_LIBS " -lm")
	endif()
endif()

# C consumers of the static C++ implementation also link its runtime.
if(APPLE)
    set(BOX2D_CXX_RUNTIME " -lc++")
elseif(UNIX OR MINGW)
    set(BOX2D_CXX_RUNTIME " -lstdc++")
else()
    set(BOX2D_CXX_RUNTIME "")
endif()
if(BUILD_SHARED_LIBS)
    string(APPEND PKGCONFIG_LIBS_PRIVATE "${BOX2D_CXX_RUNTIME}")
else()
    string(APPEND PKGCONFIG_LIBS "${BOX2D_CXX_RUNTIME}")
endif()

configure_file(box2d.pc.in box2d.pc.gen @ONLY)

# 库名和断言宏随配置变化，按配置完成替换，而不是只在 configure 时替换一次
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/$<CONFIG>/box2d.pc"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/box2d.pc.gen"
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/$<CONFIG>/box2d.pc"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
)
