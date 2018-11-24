[image1]: ./img/straight.PNG "im1"
[image2]: ./img/curve.PNG "im2"
[gif]: ./img/mpc-vid2.gif "gif1"
[image3]: ./doc/3_ModelPredictiveControlFramework.PNG "im3"

# Demo of Model Predictive Control (MPC) for Autonomous Driving

A vehicle drives autonomously in a simulated environment, with its steering angle and throttle predicted by a kinematic model. The "model predictive" controller calculates the cross tracking error and optimize the vehicle trajectory based on an approximate motion model. 

![alt text][gif]


This project is my solution to term 2, assignment 5 of the Udacity Self-Driving Car Engineer Nanodegree Program. It makes use of `Ipopt` and `CppAD` libraries to calculate the optimal trajectory (minimize error of a polynomial to the given waypoints) and the associated commands.


## Example
The vehicle drives within the simulator. Below is example of driving performance in a straight portion of road and at curvature. The maximum steering angle is fixed to `[-25, +25]`  degrees and the target speed is set to about 40mph.

![alt text][image1]
![alt text][image2]


---

## Dependencies

* cmake >= 3.5. All systems: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.



---

## Reflection
### The Model
The model considers the `(x, y)`  coordinates of the vehicle, its orientation angle `psi`, its velocity `v`, as well as the cross-track error `cte` and orientation angle error `epsi`. The output are actuators acceleration `a` and steering angle $\delta$, with acceleration/deceleration limited to [-1,1] and steering angle limited to [-25, 25] degrees. The model update current state and actuations from the previous ones as below:
![equations][image3]


In the equation `Lf` is a constant measuring the distance between the car mass and the front wheels. This value is pre-determined.

The optimum acceleration (`a`) and the steering angle ($\delta$), minimize an objective function. The objective function depends on:
* Sum of squares of `cte` and `epsi`
* Sum of squares of the difference of actuators 
* Sum of squares of the difference between two consecutive actuator values (avoid swings and sharp changes)

Each of the components in the objective function have weights calibrated manually with a trial-and-error approach.



### Timestep Length and Elapsed Duration (N & dt)
I choose `N = 10` and `dt = 0.1`. Previously, I tried `N = 20` and `dt = 0.05`. Those values define the prediction horizon, impacting optimization speed and trajectory weighting. For higher ratio `N/dt` the optimizer considers lower time steps to update the actuators. In my implementation, this update frequency performs relatively well.


### Polynomial Fitting and MPC Preprocessing
I preprocess the waypoints by mapping its coordinates to the vehicle coordinate system and then I fit a 3rd-order polynomial as below:
```cpp
for (int i = 0; i < np; i++) {
  double tx = ptsx[i] - px;
  double ty = ptsy[i] - py;
  ptsx_(i) = tx * cos(-psi) - ty * sin(-psi);
  ptsy_(i) = tx * sin(-psi) + ty * cos(-psi);
}

// Fit polynomial to the points (order 3)
auto coeffs = polyfit(ptsx_, ptsy_, 3);
```


### Model Predictive Control with Latency
The latency is set to 100 milliseconds in the [main.cpp](./src/main.cpp#L190) file, reflecting realistic conditions in the vehicle system.
```cpp
this_thread::sleep_for(chrono::milliseconds(100));
```

This latency delays the actuations. While the model output depends on the output from the previous step, with this delay, the actuations is applied to two steps before as 100 milliseconds correspond to one step in my model. The weighting I choose for the different components of the cost function handles well this latency. I believe, it is also because I have a reduced target value for speed (from 100 to 70): 
```cpp
const double ref_v = 70;
```

---
## Reference
* Project [Master Repository](https://github.com/udacity/CarND-MPC-Project)
