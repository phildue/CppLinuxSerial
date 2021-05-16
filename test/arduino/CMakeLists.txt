project(ardu-serial)

add_executable(ardu-serial main.cpp)


target_link_libraries(ardu-serial LINK_PUBLIC CppLinuxSerial )
