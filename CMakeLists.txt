cmake_minimum_required(VERSION 2.8)

project( playground )

find_package( OpenCV REQUIRED )

add_executable( playground
	src/playground.cpp
    src/serial.cpp
    )

target_link_libraries(
	playground
    ${OpenCV_LIBS}
    )
