# The Unscented Kalman Filter 

The code uses the Unscented Kalman Filter to estimate the state of a moving object 
of interest with noisy lidar and radar measurements. This code was built and tested 
on Linux but can be built on any system where the following depencies work.

The code can be run in one of the two modes. Please note the dependencies for the mode.
1. Term 2 simulator provided by Udacity is the default mode.
2. Matplotlib interface: The simulator is an overkill for the project and often slows
down the development between quick tuning cycles. Therefore, this interface has been 
implemented to provide a much quicker turn around. It uses the [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)
wrapper to draw plots.

### Simulator dependencies
* Term 2 simulator
* uWebSockets

### Matplotlib depencies
* Python 2.7 headers (`sudo apt-get install python-dev`).
The project should also work with Python 3 but this hasn't been tested.

### Common dependencies
* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4

## Build Instructions

1. Clone this repo. 
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./UnscentedKF`

If you're using the Matplotlib interface, 

1. Clone this repo with `git clone --recursive`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake -DUSE_SIMULATOR=ON .. && make` 
4. Run it: `./UnscentedKF <path to data>`

