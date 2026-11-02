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

# third-party libraries, add it as subdirectory
# fmt library
set(FMT_TEST OFF CACHE BOOL "" FORCE) # disable tests
set(FMT_DOC OFF CACHE BOOL "" FORCE) # disable docs
add_subdirectory(../tools/fmt fmt-build EXCLUDE_FROM_ALL)
#pugixml library, read/write xml file
add_subdirectory(../tools/pugixml-1.16 pugixml-build EXCLUDE_FROM_ALL)

# import 3-party libs, add header file and libs path
include_directories("../tools/")

add_executable(${prog} ${SOURCES})

if(${LINK_TYPE} STREQUAL "static")
  set(LIBS MMCPPLIB.a MMC_APP_LIB.a pthread dl rt fmt::fmt pugixml::pugixml )    
  target_link_libraries(${prog} -no-pie ${LIBS} )
else()
  set(LIBS MMCPPLIB MMC_APP_LIB pthread dl rt)
  target_link_libraries(${prog} -rdynamic -Wl,-rpath,"/mnt/usr/UserScripts:/mnt/jffs/usr/lib:/usr/lib:/lib" ${LIBS})
endif()
