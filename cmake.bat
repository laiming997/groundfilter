@echo off
SET build=E:\point_cloud\GroundFilter\filter\build
SET Makefile = Makefile
if not exist %build% (
        cd    filter
        mkdir build
    ) else (
        echo "build is already existed"
    )
cd filter
cd build 
call "E:\visual_studio2017\VC\Auxiliary\Build\\vcvarsall.bat" x64
cmake -DCMAKE_BUILD_TYPE=Debug  -G "CodeBlocks - NMake Makefiles" ../ 

