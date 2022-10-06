cmake_minimum_required(VERSION 3.7)

project(example)

add_executable(${PROJECT_NAME}

	core.h
	dron.h
	core.cpp
	dron.cpp
	main.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -std=c++1z -g -O0)
