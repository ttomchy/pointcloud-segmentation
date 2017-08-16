cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract-feature)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_definitions(-std=c++11)


add_executable(random-forest random-forest.cpp )
target_link_libraries( random-forest ${OpenCV_LIBS} )

add_executable(test-opencv test-opencv.cpp )
target_link_libraries(test-opencv ${OpenCV_LIBS} )

add_executable( test-rdf test-rdf.cpp)
target_link_libraries(test-rdf ${OpenCV_LIBS} )


add_executable( RF_letter_recog RF_letter_recog.cpp)
target_link_libraries(RF_letter_recog ${OpenCV_LIBS} )



add_executable( cpp_read_data cpp_read_data.cpp )

target_link_libraries(cpp_read_data ${OpenCV_LIBS} )



#add_executable(filter filter.cpp)
#target_link_libraries(filter ${PCL_LIBRARIES})
#add_executable(statistical-filter statistical-filter.cpp )
#target_link_libraries(statistical-filter ${PCL_LIBRARIES})

#add_executable(voxel_grid voxel_grid.cpp )
#target_link_libraries(voxel_grid ${PCL_LIBRARIES})
#

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


#add_executable (extract_indices extract_indices.cpp)
#target_link_libraries (extract_indices ${PCL_LIBRARIES})
#target_link_libraries(extract_indices ${OpenCV_LIBS} )
#



#add_executable (read_data read_data.cpp)

add_executable (cal-cov cal-cov.cpp)
target_link_libraries (cal-cov ${PCL_LIBRARIES})

#add_executable (test-eigen test-eigen.cpp)
#target_link_libraries (test-eigen ${PCL_LIBRARIES})
#
#
#add_executable (eigen2. eigen2..cpp)
#target_link_libraries (eigen2. ${PCL_LIBRARIES})

#add_executable (cluster_extraction cluster_extraction.cpp)
#target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

#
#
#
#add_executable (read_dataset read_dataset.cpp)
#target_link_libraries (read_dataset ${PCL_LIBRARIES})