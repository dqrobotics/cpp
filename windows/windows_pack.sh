#!/bin/bash
#TODO: GET THIS AUTOMATICALLY SOMEHOW
DQ_ROBOTICS_VERSION=0.1

#######################
## Visual Studio 2015
#######################
VS_2015_WIN32_FOLDER=build_2015_x86
VS_2015_WIN64_FOLDER=build_2015_x64
VS_2015_ZIP_FOLDER=dqrobotics_"$DQ_ROBOTICS_VERSION"_VS2015

mkdir -p $VS_2015_ZIP_FOLDER/lib/x86/Release
mkdir -p $VS_2015_ZIP_FOLDER/lib/x86/Debug

cp -r "$VS_2015_WIN32_FOLDER"/Release/* "$VS_2015_ZIP_FOLDER"/lib/x86/Release
cp -r "$VS_2015_WIN32_FOLDER"/Debug/* "$VS_2015_ZIP_FOLDER"/lib/x86/Debug
cp -r ../include "$VS_2015_ZIP_FOLDER"/

mkdir -p $VS_2015_ZIP_FOLDER/lib/x64/Release
mkdir -p $VS_2015_ZIP_FOLDER/lib/x64/Debug

cp -r "$VS_2015_WIN64_FOLDER"/Release/* "$VS_2015_ZIP_FOLDER"/lib/x64/Release
cp -r "$VS_2015_WIN64_FOLDER"/Debug/* "$VS_2015_ZIP_FOLDER"/lib/x64/Debug
cp -r ../include "$VS_2015_ZIP_FOLDER"/

/c/Program\ Files/7-Zip/7z.exe a -r -tzip "$VS_2015_ZIP_FOLDER".zip $VS_2015_ZIP_FOLDER

#######################
## Visual Studio 2017
#######################
VS_2017_WIN32_FOLDER=build_2017_x86
VS_2017_WIN64_FOLDER=build_2017_x64
VS_2017_ZIP_FOLDER=dqrobotics_"$DQ_ROBOTICS_VERSION"_VS2017

mkdir -p $VS_2017_ZIP_FOLDER/lib/x86/Release
mkdir -p $VS_2017_ZIP_FOLDER/lib/x86/Debug

cp -r "$VS_2017_WIN32_FOLDER"/Release/* "$VS_2017_ZIP_FOLDER"/lib/x86/Release
cp -r "$VS_2017_WIN32_FOLDER"/Debug/* "$VS_2017_ZIP_FOLDER"/lib/x86/Debug
cp -r ../include "$VS_2017_ZIP_FOLDER"/

mkdir -p $VS_2017_ZIP_FOLDER/lib/x64/Release
mkdir -p $VS_2017_ZIP_FOLDER/lib/x64/Debug

cp -r "$VS_2017_WIN64_FOLDER"/Release/* "$VS_2017_ZIP_FOLDER"/lib/x64/Release
cp -r "$VS_2017_WIN64_FOLDER"/Debug/* "$VS_2017_ZIP_FOLDER"/lib/x64/Debug
cp -r ../include "$VS_2017_ZIP_FOLDER"/

/c/Program\ Files/7-Zip/7z.exe a -r -tzip "$VS_2017_ZIP_FOLDER".zip $VS_2017_ZIP_FOLDER

#######################
## Visual Studio 2019
#######################
VS_2019_WIN64_FOLDER=build_2017_x64
VS_2019_ZIP_FOLDER=dqrobotics_"$DQ_ROBOTICS_VERSION"_VS2019

mkdir -p $VS_2019_ZIP_FOLDER/lib/x64/Release
mkdir -p $VS_2019_ZIP_FOLDER/lib/x64/Debug

cp -r "$VS_2019_WIN64_FOLDER"/Release/* "$VS_2019_ZIP_FOLDER"/lib/x64/Release
cp -r "$VS_2019_WIN64_FOLDER"/Debug/* "$VS_2019_ZIP_FOLDER"/lib/x64/Debug
cp -r ../include "$VS_2019_ZIP_FOLDER"/

/c/Program\ Files/7-Zip/7z.exe a -r -tzip "$VS_2019_ZIP_FOLDER".zip $VS_2019_ZIP_FOLDER