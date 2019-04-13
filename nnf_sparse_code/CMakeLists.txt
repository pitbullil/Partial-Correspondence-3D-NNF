cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(template_clouds)

find_package(PCL 1.8 REQUIRED)

set(MY_INCLUDE_DIR "./include")
set(GEODESIC_DIR "./geodesic")
set(GEODESIC_DIR "./geodesic/fastmarch")

set(CLASS_IMPL_FILES "y_stand.cpp fastmarchutils.cpp fastmarch.cpp pcl_fast_marching.cpp cloud_nnf_multi_visl_omp.cpp proj_namespaces.cpp cloud_analysis_visl.cpp cloud_features_visl.tpp cloud_features_visl.cpp cloud_key_points_visl.cpp cloud_registration_visl.cpp cloud_visualization_visl.cpp cloud_io_visl.cpp cloud_registration.cpp misc_utils.cpp meshgraph.cpp cloud_surface_visl.cpp")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
include_directories(${PCL_INCLUDE_DIRS} ${MY_INCLUDE_DIR} ${GEODESIC_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")

add_executable (3DDIS_OMP 3DDIS_OMP.cpp y_stand.cpp fastmarchutils.cpp fastmarch.cpp pcl_fast_marching.cpp cloud_nnf_multi_visl_omp.cpp proj_namespaces.cpp cloud_analysis_visl.cpp cloud_features_visl.tpp cloud_features_visl.cpp cloud_key_points_visl.cpp cloud_registration_visl.cpp cloud_visualization_visl.cpp cloud_io_visl.cpp cloud_registration.cpp misc_utils.cpp meshgraph.cpp cloud_surface_visl.cpp)
target_link_libraries (3DDIS_OMP ${PCL_LIBRARIES})