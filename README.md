# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project I utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This project includes source files included in folder src:

* main.cpp -  establishes the connection with simulator, reads out input data and writes the vehicle position estimateted by UKF

* ukf.cpp - initializes and desribes the system states and covariances. Defines and calls the prediction and update steps. 

* ukf.h - declares the ukf class and it variables and methods

* tools.cpp - calculates RMSE

* tools.h - declares tools class and its methods


[//]: # (Image References)
[image1]: ./ukf_meas.jpg
[image2]: ./nis_radar.JPG
[image3]: ./nis_laser.JPG

## Result
I obtained following resultw when using both Radar and Lidar data for the position and velocity estimation:

* red circles - position derived lidar measuremets

* blue circles - position derived from radar mesurements ( converted from ro, theta => x,y)

* green triangles - position estimated by UKF

![][image1] 

NIS calculation gives following results:

Radar NIS:
![][image2] 


Lidar NIS:
![][image3] 

NIS for both sensors show consistent results corresponding to their DoF.
