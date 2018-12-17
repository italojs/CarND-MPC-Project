# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distroshttps://visualstudio.microsoft.com/pt-br/team-services/
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc` or `./makeAndrun.sh`.

```
> ./makeAndrun.sh
[100%] Built target mpc
Listening to port 4567
```

Now the MPC controller is running and listening on port 4567 for messages from the simulator. Next step is to open Udacity's simulator and choose the project 5 (MPC Controller)

Here has a sht video with the final parameters [./videos/final-parameters.mov](./video/final-parameters.mov).

# [Rubic](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile.

The code compiles without errors or warnings. No modifications were done on the provided setup.

## Implementation

### The Model

Kinematic model:

``` c++
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

The objective is to find the steering angle(`delta`) and the acceleration (`a`) in the way it will minimize an objective function that is the combination of different factors:

### Timestep Length and Elapsed Duration (N & dt)

#### What's define the predict horizon?
The time interval(`dt`) and the number of points(`N`) define the prediction horizon.

#### The number of points impact something?
The points quantity impacts the controller performance. 

#### How did I define the 'N' and 'dt'?
I tried to keep the horizon around the same time the waypoints were on the simulator. With too many points the controller starts to run slower. I decided to leave them fixed to 10 and 100 milliseconds after tried with `N` from 10 to 20 and `dt` 100 to 500 milliseconds, and to have a better result tuning the other parameters too.

### Polynomial Fitting and MPC Preprocessing

The waypoints are transformed to the car coordinate system atarts at [./src/main.cpp](./src/main.cpp#L86). Then the waypoints area is tranformed with a 3rd-degree polynomial.

This polynomial coefficients are used to calculate the `cte` and `epsi`.

### Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code could be found [here](./src/main.cpp#L99).

The latency of 100ms is added before sending actuations to the simulator. 

## Simulation

### The vehicle must successfully drive a lap around the track.

Here is a video with the final parameters: [./videos/final-parameters.mov](./video/final-parameters.mov).
