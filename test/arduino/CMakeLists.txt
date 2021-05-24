project(ardu-serial)

add_executable(ardu-serial main.cpp SerialProtocol.h SerialProtocol.cpp)


target_link_libraries(ardu-serial LINK_PUBLIC CppLinuxSerial )
