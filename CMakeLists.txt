cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(CcTrace CUDA CXX)
add_subdirectory(image)

add_subdirectory(${CMAKE_SOURCE_DIR}/nvbit/mem_trace)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(meminf SHARED meminf_default.cc)
add_executable(rt trace.cu bvh.cc reader/ply.cc)
target_include_directories(rt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(rt PUBLIC cxx_std_11)
target_link_libraries(rt image meminf)
target_compile_options(rt PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-gencode arch=compute_75,code=sm_75>)

find_package(OpenMP)


add_executable(rtNC trace.cu bvh.cc reader/ply.cc)
target_include_directories(rtNC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(rtNC PUBLIC cxx_std_11)
target_link_libraries(rtNC image meminf)
target_compile_options(rtNC PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:-Xptxas -dlcm=cg>)
target_compile_options(rtNC PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-gencode arch=compute_75,code=sm_75>)

if(OpenMP_CXX_FOUND)
    target_link_libraries(rt OpenMP::OpenMP_CXX)
    target_link_libraries(rtNC OpenMP::OpenMP_CXX)
endif()
