cmake_minimum_required(VERSION 2.8)

project( visionNetLights )

find_package( OpenCV REQUIRED )

add_executable( visionForNetLights main.cpp )

target_link_libraries(
	visionForNetLights
       	${OpenCV_LIBS}
       	)
