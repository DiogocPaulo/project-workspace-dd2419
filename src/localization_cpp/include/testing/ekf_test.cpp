#include <iostream>
#include "ekf.hpp"  // Include the EKF class header

int main() {
    // Create EKF object
    Localization::EKF ekf;

    // Print initial state
    std::cout << "Initial State: " << std::endl;
    std::cout << ekf.getState().transpose() << std::endl;

    // Define some test inputs for the prediction and correction steps
    double delta_x = 0.1;  // Change in x
    double delta_y = 0.1;  // Change in y
    double delta_theta = 0.1;  // Change in orientation (theta)
    double delta_v = 0.8;  // Change in linear velocity
    double delta_w = 0.01;  // Change in angular velocity
    double dt = 0.1;  // Time step

    // Perform prediction step based on encoder data
    ekf.predict(delta_x, delta_y, delta_theta, delta_v, delta_w, dt);

    // Print state after prediction
    std::cout << "State after prediction: " << std::endl;
    std::cout << ekf.getState().transpose() << std::endl;

    // Simulate IMU measurements for correction (angular velocity, linear velocity)
    double imu_angular_velocity_z = 0.03;   // Example linear velocity from IMU

    // Perform correction step based on IMU measurement
    ekf.correct(imu_angular_velocity_z, dt);

    // Print state after correction
    std::cout << "State after correction: " << std::endl;
    std::cout << ekf.getState().transpose() << std::endl;

    return 0;
}
