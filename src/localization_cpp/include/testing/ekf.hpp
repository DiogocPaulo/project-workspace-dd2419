#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace Localization {

class EKF {
public:
    Eigen::VectorXd state_; // State vector [x, y, theta, v, w]
    Eigen::MatrixXd P_;     // State covariance matrix
    Eigen::MatrixXd Q_;     // Process noise covariance (encoder noise)
    Eigen::MatrixXd R_;     // Measurement noise covariance (IMU noise)
    Eigen::MatrixXd U_;     // Control input noise covariance (encoder noise)

    // Constructor
    EKF() {
        // Initialize state vector [x, y, theta, v, w]
        state_ = Eigen::VectorXd(5);
        state_.setZero();

        // Initialize covariance matrices
        P_ = Eigen::MatrixXd::Identity(5, 5) * 0.1; // Initial uncertainty
        Q_ = Eigen::MatrixXd(5, 5);                 // Process noise
        Q_ << 0.01, 0.0,  0.0,  0.0,   0.0,    // x
              0.0,  0.01, 0.0,  0.0,   0.0,    // y
              0.0,  0.0,  0.01, 0.0,   0.0,    // theta
              0.0,  0.0,  0.0,  0.05,  0.0,    // v (higher during accel)
              0.0,  0.0,  0.0,  0.0,   0.05;   // w (higher during turns)
        R_ = Eigen::MatrixXd(2, 2);                 // Measurement noise (IMU)
        R_ << 0.1,  0.0,                           // v (IMU less trusted)
              0.0,  0.1;                           // w (IMU less trusted)
        U_ = Eigen::MatrixXd::Identity(2, 2) * 0.01; // Control noise (encoders)
    }

    // Predict step based on encoder data
    void predict(double delta_x, double delta_y, double delta_theta, double delta_v, double delta_w, double dt) {
        // Motion model Jacobian
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 2) = -delta_y * sin(state_(2));  // ∂x/∂theta
        F(1, 2) = delta_x * cos(state_(2));   // ∂y/∂theta
        F(0, 3) = dt * cos(state_(2));        // ∂x/∂v
        F(1, 3) = dt * sin(state_(2));        // ∂y/∂v
        F(2, 4) = dt;                         // ∂theta/∂w

        // Update state estimate (nonlinear motion model)
        double new_x = state_(0) + delta_x * cos(state_(2)) - delta_y * sin(state_(2));
        double new_y = state_(1) + delta_x * sin(state_(2)) + delta_y * cos(state_(2));
        state_(0) = new_x;
        state_(1) = new_y;
        state_(2) += delta_theta;

        // Update velocities (assuming delta_v and delta_w are instantaneous)
        state_(3) = delta_v;  // Linear velocity from encoders
        state_(4) = delta_w;  // Angular velocity from encoders

        // Predict covariance
        P_ = F * P_ * F.transpose() + Q_;

        // Optionally add control noise (if modeling encoder uncertainty separately)
        // P_ += U_;  // Uncomment if using control input noise
    }

    // Correct step based on IMU measurement (correcting theta and angular velocity)
    // Correct step based on IMU measurement (correcting theta and angular velocity)
    void correct(double angular_velocity_imu, double dt) {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Correcting with IMU data..." << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Calculate the IMU theta using only the angular velocity data from the IMU
        double imu_theta = (state_(2) - state_(4) * dt) + (angular_velocity_imu * dt);

        // Measurement vector: IMU angular velocity and IMU orientation (theta)
        Eigen::VectorXd z(2);
        z << angular_velocity_imu, imu_theta;

        std::cout << "IMU theta: " << imu_theta << std::endl;
        std::cout << "IMU angular velocity: " << angular_velocity_imu << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Expected measurement vector from current state
        Eigen::VectorXd z_pred(2);
        z_pred << state_(4), state_(2);  // Current state contains w and theta

        // Innovation (residual)
        Eigen::VectorXd y = z - z_pred;

        std::cout << "Measurement vector: " << z.transpose() << std::endl;
        std::cout << "Predicted measurement vector: " << z_pred.transpose() << std::endl;
        std::cout << "Innovation (residual): " << y.transpose() << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Measurement Jacobian H (sensitive to w and theta)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
        std::cout << "H rows: " << H.rows() << ", H cols: " << H.cols() << std::endl;
        H(0, 4) = 1.0;  // ∂w/∂w
        H(1, 2) = 1.0;  // ∂theta/∂theta

        // Innovation covariance
        Eigen::MatrixXd R = R_;

        // Print matrices for debugging
        std::cout << "----------------------------------------" << std::endl;

        std::cout << "P matrix: " << std::endl;
        std::cout << P_ << std::endl;
        std::cout << "H matrix: " << std::endl;
        std::cout << H << std::endl;
        std::cout << "R matrix: " << std::endl;
        std::cout << R << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        Eigen::MatrixXd S = H * P_ * H.transpose() + R;

        std::cout << "S matrix: " << std::endl;
        std::cout << S << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Kalman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        std::cout << "Kalman gain: " << std::endl;
        std::cout << K << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Update state estimate using Kalman gain
        state_ += K * y;

        std::cout << "Updated state: " << std::endl;
        std::cout << state_.transpose() << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Update state covariance matrix
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
        P_ = (I - K * H) * P_;

        std::cout << "Updated covariance matrix: " << std::endl;
        std::cout << P_ << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // Optionally, normalize theta to [-π, π] after correction
        while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
        while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;
    }

    // Get the current state
    Eigen::VectorXd getState() const {
        return state_;
    }

    // Set process noise covariance matrix
    void setProcessNoise(const Eigen::MatrixXd& Q) {
        Q_ = Q;
    }

    // Set measurement noise covariance matrix
    void setMeasurementNoise(const Eigen::MatrixXd& R) {
        R_ = R;
    }

    // Set control input noise covariance matrix
    void setControlNoise(const Eigen::MatrixXd& U) {
        U_ = U;
    }

private:
    double prev_imu_theta_ = 0.0;
    bool first_imu_reading_ = true; 
};

}  // namespace Localization