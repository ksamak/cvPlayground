cmake_minimum_required(VERSION 2.8)

project( visionNetLights )

find_package( OpenCV REQUIRED )

add_executable( visionForNetLights
	playground.cpp
    16x16/test/test_serial.cpp
    )

target_link_libraries(
	visionForNetLights
       	${OpenCV_LIBS}
       	)

add_executable( faceRecognition
    faceRecognition.cpp
    )

target_link_libraries(
    faceRecognition
    ${OpenCV_LIBS}
       	)
