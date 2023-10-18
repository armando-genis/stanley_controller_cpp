# stanley_controller_cpp
 santly control in c++


![Demo GIF](https://github.com/armando-genis/stanley_controller_cpp/blob/main/images/car_stanley.gif)

## Installation

 ```bash
  sudo apt install libeigen3-dev
  sudo apt install python3-matplotlib python3-dev
  wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h
```

## Run Locally

```bash
g++ -I/usr/include/eigen3 -I/usr/include/python3.8 stanley_controller.cpp -o stanley_controller -lpython3.8
g++ -I/usr/include/eigen3 -I/usr/include/python3.8 BicycleModel.cpp stanley_controller.cpp -o stanley_controller -lpython3.8
./stanley_controller


g++ -I/usr/include/eigen3 -I/usr/include/python3.8 BicycleModel.cpp Animation.cpp Linear_Interpolation.cpp -o stanley_animation -lpython3.8

g++ -I/usr/include/eigen3 -I/usr/include/python3.10 BicycleModel.cpp Animation.cpp Linear_Interpolation.cpp -o stanley_animation -lpython3.10

g++ -I/usr/include/eigen3 -I/usr/include/python3.10 BicycleModel.cpp StanleyController.cpp Animation.cpp Linear_Interpolation.cpp -o stanley_animation -lpython3.10

g++ -I/usr/include/eigen3 -I/usr/include/python3.8 BicycleModel.cpp StanleyController.cpp Animation.cpp Linear_Interpolation.cpp CubicSpline1D.cpp -o stanley_animation -lpython3.8


  ./stanley_animation



```


http://www.gnuplot.info/