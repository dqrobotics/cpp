#!/bin/bash
CALL_CMAKE="cmake -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE -DCMAKE_TOOLCHAIN_FILE=/d/git/vcpkg/scripts/buildsystems/vcpkg.cmake -DBUILD_SHARED_LIBS=TRUE -H. "

cd ..
## Visual Studio 2015
mkdir -p windows/build_2015_x64
$CALL_CMAKE -G'Visual Studio 14 2015 Win64' -Bwindows/build_2015_x64

mkdir -p windows/build_2015_x86
$CALL_CMAKE -G'Visual Studio 14 2015' -Bwindows/build_2015_x86

## Visual Studio 2017
mkdir -p windows/build_2017_x64
$CALL_CMAKE -G'Visual Studio 15 2017 Win64' -Bwindows/build_2017_x64
mkdir -p windows/build_2017_x86
$CALL_CMAKE -G'Visual Studio 15 2017' -Bwindows/build_2017_x86

## Visual Studio 2019
mkdir -p windows/build_2019_x64
$CALL_CMAKE -G'Visual Studio 16 2019' -Bwindows/build_2019_x64
