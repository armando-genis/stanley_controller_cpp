# stanley_controller_cpp
 santly control in c++


## Installation

 ```bash
  sudo apt install libeigen3-dev
  sudo apt install python3-matplotlib python3-dev
  wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h
```

## Run Locally

```bash
  g++ -I/usr/include/eigen3 -I/usr/include/python3.8 stanley_controller.cpp -o stanley_controller -lpython3.8
  ./stanley_controller

```