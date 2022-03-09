echo "Setting up vcpkg..."
cd C:\vcpkg
.\bootstrap-vcpkg.bat
vcpkg integrate install
echo "Installing eigen3..."
.\vcpkg install eigen3:x64-windows
echo "Adding symlink to vcpkg..."
cmd /c mklink /d c:\Tools\vcpkg c:\vcpkg
cd ~
