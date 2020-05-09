# dqrobotics-cpp WINDOWS 

# Users

Use the Python version.

# Developers

**USE AT YOUR OWN RISK, THERE IS NO SUPPORT**

## Compiler/IDE
Install [Visual Studio Community 2019](https://visualstudio.microsoft.com/vs/).

## Install vcpkg (Using Powershell)

```powershell
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install
```

## Install Eigen3

```powershell
.\vcpkg install eigen3:x64-windows
```

## Compile 
Open the project in Visual Studio Community 2019 and compile using the CMakeLists.txt.

## Link
Add the compiled library in your project.

## Useful info
Python [WindowsCompilers](https://wiki.python.org/moin/WindowsCompilers)