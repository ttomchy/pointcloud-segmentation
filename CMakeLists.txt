cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(filter-test)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(filter filter.cpp)
#target_link_libraries(filter ${PCL_LIBRARIES})
#add_executable(statistical-filter statistical-filter.cpp )
#target_link_libraries(statistical-filter ${PCL_LIBRARIES})

#add_executable(voxel_grid voxel_grid.cpp )
#target_link_libraries(voxel_grid ${PCL_LIBRARIES})


#add_executable(remove_outliers remove_outliers.cpp )
#target_link_libraries(remove_outliers ${PCL_LIBRARIES})

#add_executable(split split.cpp )
#target_link_libraries(split ${PCL_LIBRARIES})


#add_executable(passthrough passthrough.cpp )
#target_link_libraries(passthrough ${PCL_LIBRARIES})


#add_executable(cylinder_segmentation cylinder_segmentation.cpp )
#target_link_libraries(cylinder_segmentation ${PCL_LIBRARIES})


#add_executable(planar_segmentation planar_segmentation.cpp )
#target_link_libraries(planar_segmentation ${PCL_LIBRARIES})

#add_executable (cylinder_segmentation cylinder_segmentation.cpp)
#target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})


add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})



add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})