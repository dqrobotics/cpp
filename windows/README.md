# dqrobotics-cpp WINDOWS pre-alpha

# Users

Get one of the released versions.
**USE AT YOUR OWN RISK, THERE IS NO SUPPORT**

# Developers

Install vcpkg (Using git bash)

1) git clone https://github.com/Microsoft/vcpkg.git
2) cd vcpkg
3) ./bootstrap-vcpkg.sh
4) ./vcpk.exe integrate install

Install Eigen3

1) ./vcpkg.exe install eigen3

Configure VS solutions

1) Update windows_cmake_config.sh with -DCMAKE_TOOLCHAIN_FILE=[your vcpkg path]\scripts
2) ./windows_cmake_config.sh

Open solutions and build them in Debug and Release modes with the relevant VS versions

Pack!

1) ./windows_pack.sh
