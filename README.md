# **PID Control Project**

## Goals

In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement
a PID controller in C++ to maneuver the vehicle around the track.

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering
angle.

[//]: # "Image References"

[image1]: ./twiddle.png "Twiddle"

## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Compilation

#### 1. Your code should compile.  

Code must compile without errors with cmake and make.

`CMakeLists.txt` has been modified to properly compile the project.

### Implementation

#### 1. The PID procedure follows what was taught in the lessons.

The PID is implemented as described in the lessons and Twiddle algorithm is implemented for automatically tuning PID coefficients.

### Reflection

#### 1. Describe the effect each of the P, I, D components had in your implementation.

* The P component regulates the control input proportionally to the CTE.
* The I component regulates the control input proportionally to the accumulation of the error over time. It is useful to 
  compensate systematic errors.
* The D component regulates the control input proportionally to the difference of the last two CTE values. It is useful to
  avoid oscillations.

#### 3. Describe how the final hyperparameters were chosen.

* The final hyperparameters for the speed PID have been tuned through Twiddle.
  ![alt text][image1]
* The ones related to the angle have been tuned manually because of the accidents that acted as noises on the input of the 
controller and did not permit to automatically tune coefficients. I used a trial and error approach, modifying the coefficients
based on what i learned in the lessons.

### Simulation

#### 1. The vehicle must successfully drive a lap around the track.
No tire leaves the drivable portion of the track surface, the car does not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe.
