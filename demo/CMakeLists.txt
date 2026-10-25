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

# Add an if clause to check the value of the variable
if(${LINK_TYPE} STREQUAL "static")
  set(LIBS MMCPPLIB.a MMC_APP_LIB.a pthread dl rt)    
else()
  set(LIBS MMCPPLIB MMC_APP_LIB pthread dl rt)
endif()
add_executable(${prog} ${SOURCES})

if(${LINK_TYPE} STREQUAL "static")
  set(LIBS MMCPPLIB.a MMC_APP_LIB.a pthread dl rt)    
  target_link_libraries(${prog} -no-pie ${LIBS})
else()
  set(LIBS MMCPPLIB MMC_APP_LIB pthread dl rt)
  target_link_libraries(${prog} -rdynamic -Wl,-rpath,"/mnt/usr/UserScripts:/mnt/jffs/usr/lib:/usr/lib:/lib" ${LIBS})
endif()
