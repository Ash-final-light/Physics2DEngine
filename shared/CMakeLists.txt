add_library(shared STATIC utils.cpp determinism.cpp human.cpp)
target_compile_features(shared PRIVATE cxx_std_20)
target_include_directories(shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(shared PUBLIC box2d)
