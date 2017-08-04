# CarND-Controls-PID

PID or proportional–integral–derivative controller is widely used in industrial control systems and a variety of other applications requiring continuously modulated control. 

In this project pid control is used to control steering angle & throttle of the vehicle by continuously calculating pid errors, given the distance between the actual car position on the road and a reference trajectory, known as cross-track error (cte) to minimize errors:

* p_error: cross track error
* d_error: as the difference between a desired setpoint and a measured process variable
* i_error: 

I did manual tuning using [wikipedia](https://en.wikipedia.org/wiki/PID_controller), by first setting Kp, Kd & Ki values to zero. I increased Kp until the output of the loop oscillates, then started tweaking Ki to correct offset considering that high Ki would cause instability. Finally, I increase Kd until the loop is acceptably quick to reach its reference after a load disturbance considering that too much Kd will cause excessive response and overshoot. 
However fast tuning usually overshoots slightly to reach the setpoint more quickly which is very noticable when car is adjusting its steering-angle on entering or exiting turns; and I believe combining the results with machine learning tehcniques used in behaviroal cloning project would achive optimum result and correct flaws in both approaches.

I also used pid for adjusting throttle to speed up or brake, however I had to set conditions for checking cte values and sharp steering-angles to lower the throttle when error/curve is increasing for a smoother driving.

## My twiddle expermient and why it failed

I tried using the twiddle code (taught in the course) and there were a couple of obstacles using it in a realtime scenario.
For the very first error-avg (as best average error) robot is moved for 100 iterations and error is averaged, then one of the Kp/Kd/Ki is modified to run the 100-iteration experiment on the robot again and reseting total_error, the same process is repeated for the next 100 iterations over and over again. Applying it to simulated car, 100 iteration is enough to increase cte and go off the road (fewer iterations didn't resolve the issue either and I wasn't able to tune dp values, code is commented out in pid.cpp).
I think twiddle is good for an offline experiment to adjust K values and re-run the car on same path multiple times rather than running it on a road wihtout similar starting points and curves, then running it in realtime. 


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
