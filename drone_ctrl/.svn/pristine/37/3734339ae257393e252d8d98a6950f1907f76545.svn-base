cmake_minimum_required(VERSION 2.6)
project(mis)
#find_package(raspicam REQUIRED)
find_package(raspicam)
if (raspicam_FOUND)
  set(EXTRA_CC_FLAGS " -O2 -g0")
  message("CMAKE found raspberry camere module")
  add_definitions(-Draspicam_CV_LIBS)
else()
  message("CMAKE set to compile without rapberry pi camera modules")
  set(EXTRA_CC_FLAGS "-DNO_PI_CAM -O0 -g2")
endif()
find_package(OpenCV REQUIRED )
find_package(Threads REQUIRED)
# NB the file FindReadline.cmake need to be copied to where cmake
#    expects to find such files, e.g.: /usr/share/cmake-3.16/Modules/
#    the file is in this directory: sudo cp FindReadline.cmake /usr/share/cmake-3.16/Modules/
#    also install readline: sudo apt install libreadline-dev
find_package(Readline REQUIRED)
# this works on some platforms as an alternative - the target_link_libraries line need change too
#find_package(PkgConfig REQUIRED)
#pkg_search_module(PKG_READLINE REQUIRED readline)
include_directories(${OpenCV_INCLUDE_DIRS} data lib src)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-stringop-truncation -pedantic -std=c++11 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")
add_executable(mission 
               main.cpp 
               lib/urun.cpp 
               lib/ucamera.cpp 
               lib/ubridge.cpp 
               lib/utime.cpp 
               lib/tcpCase.cpp 
               lib/udata.cpp 
               lib/udatabase.cpp 
               lib/userverclient.cpp 
               lib/userverport.cpp 
               lib/uoptitrack.cpp 
               lib/ulogfile.cpp 
               lib/upublish.cpp 
               lib/urelay.cpp 
               lib/uunhandled.cpp
               data/ujoy.cpp 
               data/ustate.cpp 
               data/upose.cpp 
               data/uaccgyro.cpp 
               data/uaruco.cpp 
               data/uoptidata.cpp 
               data/uteensy.cpp 
               data/ugps.cpp
               data/ugamepad.cpp
               src/umission.cpp)
target_link_libraries(mission ${OpenCV_LIBS} ${raspicam_CV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${Readline_LIBRARY})
#target_link_libraries(mission ${OpenCV_LIBS} ${raspicam_CV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${PKG_READLINE_LDFLAGS})
install(TARGETS mission RUNTIME DESTINATION bin)
