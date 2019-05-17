#!/bin/bash

## Visual Studio 2015
mkdir -p build_2015_x64
cmake -H. -G'Visual Studio 14 2015 Win64' -Bbuild_2015_x64

mkdir -p build_2015_x86
cmake -H. -G'Visual Studio 14 2015' -Bbuild_2015_x86

## Visual Studio 2017
mkdir -p build_2017_x64
cmake -H. -G'Visual Studio 15 2017 Win64' -Bbuild_2017_x64
mkdir -p build_2017_x86
cmake -H. -G'Visual Studio 15 2017' -Bbuild_2017_x86

## Visual Studio 2019
mkdir -p build_2019_x64
cmake -H. -G'Visual Studio 16 2019' -Bbuild_2019_x64
