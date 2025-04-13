cmake_minimum_required(VERSION 3.0.0)

project(UserProgram VERSION 0.1.0)


# Define a local variable
set(LINK_TYPE "static")
#set(LINK_TYPE "dynamic")

# Print the value of the variable
message("The Link Type is: ${LINK_TYPE}")

# set target program name as directory name: launcher uses it
get_filename_component(prog ${CMAKE_CURRENT_SOURCE_DIR} NAME)

if(${maestroVariant} MATCHES "^pmas$")
  set(prog ${prog}.pexe)
else()
  set(prog ${prog}.texe)
endif()

file(GLOB SOURCES *.cpp *.c)

# import 3-party libs, add header file and libs path
include_directories(/home/elmo/dev/curses-linux/include)
include_directories(/home/elmo/dev/curses-linux/include/ncurses)
link_directories(/home/elmo/dev/curses-linux/lib)

add_executable(${prog} ${SOURCES})

if(${LINK_TYPE} STREQUAL "static")
  set(LIBS ncurses)    
  target_link_libraries(${prog} -no-pie ${LIBS})
else()
  set(LIBS MMCPPLIB MMC_APP_LIB pthread dl rt)
  target_link_libraries(${prog} -rdynamic -Wl,-rpath,"/mnt/usr/UserScripts:/mnt/jffs/usr/lib:/usr/lib:/lib" ${LIBS})
endif()
