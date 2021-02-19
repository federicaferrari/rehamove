cmake_minimum_required(VERSION 3.5)

project(stimulation LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(libsmpt)



add_executable(main
    main.cpp




    )


#target_link_libraries(main PRIVATE Qt5::Widgets ${CMAKE_SOURCE_DIR}/libsmpt.a )

add_library(libsmpt STATIC IMPORTED)
set_target_properties(libsmpt PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/libsmpt.a)

target_link_libraries(main PRIVATE Qt5::Widgets libsmpt
    Qt5::Core)
