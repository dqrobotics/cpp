##########################################################################
[Ubuntu/PPA]
##########################################################################
Install from the ppa:

  sudo add-apt-repository ppa:dqrobotics-dev/release
  sudo apt-get update
  sudo apt-get install libdqrobotics

##########################################################################
Compile from source
##########################################################################
In this folder, run:

  mkdir -p build
  cd build
  cmake ..
  make
