cmake_minimum_required(VERSION 3.16)

project(simple_cmake LANGUAGES C)

add_executable(simple_cmake test00.c)

include(GNUInstallDirs)
install(TARGETS simple_cmake
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
