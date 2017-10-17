# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Rubric points

### The Model
A simple non-linear kinematic vehicle model was used in this project. The model state is represented by position, speed and heading of the vehicle and acceleration/braking and steering control signals are used.

The main model equations are:
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where x and y represent the car position, `psi` its heading direction, `v` its velocity, `cte` its cross-track error and `epsi` its orientation error. `dt` is the measurement timestep, `Lf` is provided as a parameter and denotes the distance between the center of mass of the vehicle and the front wheels, giving a measure of its handling.

At every iteration a new state vector is provided by the simulator and the model calculates the best trajectory for the next N time steps (i.e. the one minimizing the cost function). This cost function is the sum of different costs associated to different errors and weights that can be tuned. In particular: cross-track error, heading error, difference from reference velocity, actuator values and their differentials were taken into account.

The weights, set in file `MPC.h`, were tuned to:
```
const unsigned int w_cte = 1;
const unsigned int w_epsi = 1;
const unsigned int w_v = 1;
const unsigned int w_delta = 1000;
const unsigned int w_a = 10;
const unsigned int w_ddelta = 700;
const unsigned int w_da = 1;
```
in such a way that a decent vehicle handling could be achieved up a target speed of 80mph. At lower speed, the `w_delta` can be set to 1, resulting in a better trajectory planning.

The control signals belonging to the first element of the iteration (actually in this case, to counteract latency, the one at time t+2) is then sent to the simulator. All following points are discarded and, at the next step, the entire operation is repeated.

### Timestep Length and Elapsed Duration (N & dt)
The product N * dt defines the prediction horizon. The compromise here is between short prediction horizons, resulting in more responsive but "blind" controllers, and long prediction horizons, resulting in smoother and "well planning" but somehow slow and lagging controllers.
A timestep of 50ms (along with a max_cpu_time of 50ms) was chosen empirically. The number of points N = 15 was chosen as a compromise as smaller N (e.g. 10) didn't provide enough forward planning while higher ones (e.g. 20) didn't allow the car to slalom quickly between close curves.
Also, a timestep of 50ms, coupled with a latency of 100ms, implies that the first 2 steps are actually trated differently (see dedicated section).

### Polynomial Fitting and MPC Preprocessing
A 3rd order polynomial is used as it is the lowest order giving the possibility to manage "S-shaped" conditions.
Coordinates were first pre-processed to convert them from map coordinates to car coordinates. This also helps simplifying the equations as the car position becomes (0,0). The code for this transformation is in function `transformMap2Car`:
```
double dx = ptsx[i] - x;
double dy = ptsy[i] - y;
pts_car(0,i) =  cos(psi) * dx + sin(psi) * dy;
pts_car(1,i) = -sin(psi) * dx + cos(psi) * dy;
```
while the state becomes
```
state << 0, 0, 0, v, cte, epsi;
```

### Model Predictive Control with Latency

A reaction time (latency) of 100ms was included in the vehicle model as part of the problem and needed to be accounted for. If not treated properly, this can lead to unwanted oscillations and weird trajectories due to the non-timely responses of the control.

I approached this problem by constraining the control values `delta` and `a` to their previous values for the duration of the latency (in this case, `latency_steps = 2`). Over this time, the other parameters of the model are constrained as usual and free to concur to the model optimization.
After the optimization, the control values used to control the vehicle are those `latency_steps` ahead:

```
Eigen::VectorXd state(6);
state << 0, 0, 0, v, cte, epsi;
ExtendedState solution = mpc.Solve(state, coeffs);

double steer_value = solution.delta.at(latency_steps);
double throttle_value = solution.a.at(latency_steps);
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
