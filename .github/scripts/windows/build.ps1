mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
#Not used, but good to keep in mind -DCMAKE_INSTALL_PREFIX="$ENV:UserProfile\dqrobotics" ..
cmake --build . --config Release
cd ..
