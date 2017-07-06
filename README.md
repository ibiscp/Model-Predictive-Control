# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Project discussion

### The Model

The state of the vehicle is given by the following equations:

<p align="center"><img src="images/states.png"/></p>

With the following description:

* X position
* Y position
* Orientation angle psi (depends on the steering angle)
* Velocity (depends on the acceleration)
* Cross-track error
* Psi error (depends on the steering angle)

For the actuators, two outputs are used, steering angle and acceleration, both used in the formulas above.

### Timestep Length and Elapsed Duration (N & dt)

For this project, the value chosen for N is 10 and dt is 0.1, which were suggested by Udacity. Changing these values made the car go out of the track, in order to avoid it, it would be necessary to recalibrate the error parameters.

### Polynomial Fitting and MPC Preprocessing

The waypoints are transformed to the vehicle coordinate using the following formula:

```cpp
for (size_t i = 0; i < ptsx.size(); i++) {
    x_diff = ptsx[i] - px;
    y_diff = ptsy[i] - py;

    pts_x_car[i] = x_diff * cos(psi) + y_diff * sin(psi);
    pts_y_car[i] = y_diff * cos(psi) - x_diff * sin(psi);
}
```

### Model Predictive Control with Latency

We know that the actuators does not execute instantly in a real car, it takes some time to the message propagate and the actuator execute the command. So, it is necessary to simulate this latency to make the model more realistic. In order to make it in this project, the latency was set to 100 miliseconds.

knowing the latency value (which can be measured), it is necessary to use this value to update the state vector before calling the solve function.

```cpp
px = px + v * cos(psi) * dt;
py = py + v * sin(psi) * dt;
psi = psi + v * steer_value / Lf * dt;
v = v + throttle_value * dt;
ptsx[0] = px; // adjust so [0]  will now be zero in transform
ptsy[0] = py; // adjust so [0]  will now be zero in transform
double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] *pow(px,2));
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Run it: `./run.sh`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.
