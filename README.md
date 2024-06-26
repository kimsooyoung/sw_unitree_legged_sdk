# v3.3.1

## This repo contains more advanced examples for quadruped robot controls (read below examples)

The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: Aliengo, A1(sport_mode >= v1.19)

not support robot: Laikago, Go1.

### Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher / 1.5.0 for ubuntu 22.04)
```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Usage
Run examples with 'sudo' for memory locking.
