# dqrobotics-cpp
The DQ Robotics library in C++

#Installation in Ubuntu 16.04 or Ubuntu 18.04

Choose one of the approaches below, **DO NOT ADD BOTH STABLE AND DEVELOPMENT REPOSITORIES**.

## Stable PPA

The Stable PPA is the version most users should stick to. We try to guarantee backwards compatibility between Stable versions as much as possible

You can add it to your ubuntu repository and install it with the following commands
```
sudo add-apt-repository ppa:dqrobotics-dev/release
sudo apt-get update
sudo apt-get install libdqrobotics
```

After that, updates will be delivered together with your regular ubuntu updates
```
sudo apt-get update
sudo apt-get upgrade
```

## Development PPA

The Development PPA has daily builds of the library including latest additions and fixes. The code is unstable and there is no guarantee of backwards compatibility between Unstable version of the library.

You can add it to your ubuntu repository and install it with the following commands
```
sudo add-apt-repository ppa:dqrobotics-dev/development
sudo apt-get update
sudo apt-get install libdqrobotics
```

After that, updates will be delivered together with your regular ubuntu updates
```
sudo apt-get update
sudo apt-get upgrade
```

## Compiling from source in Ubuntu

You shouldn't be doing this, but if you really have to, you can build the dqrobotics shared library as follows
```
sudo apt-get install git cmake libeigen3-dev
git clone https://github.com/dqrobotics/cpp.git
cd git
mkdir build
cd build
cmake ..
make
```

## Compiling in another OS (Unsupported)

Adjust the commands above (Compiling from source in Ubuntu), including installing git, cmake, and eigen3 for the target OS. 

