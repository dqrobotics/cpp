# dqrobotics-cpp WINDOWS pre-alpha

# Users

Get one of the released versions.
**USE AT YOUR OWN RISK, THERE IS NO SUPPORT**

# Developers

Install vcpkg (Using git bash)

1- git clone https://github.com/Microsoft/vcpkg.git
2- cd vcpkg
3- ./bootstrap-vcpkg.sh
4- ./vcpk.exe integrate install

Install Eigen3

1- ./vcpkg.exe install eigen3

Update windows_cmake_config with -DCMAKE_TOOLCHAIN_FILE=[your vcpkg path]\scripts
