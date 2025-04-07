#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

class EKF {
public:
    // State vector [x, y, theta]
    Eigen::Vector3d state_;

    // Covariance matrix
    Eigen::Matrix3d P_;

    // Process noise covariance (encoder noise)
    Eigen::Matrix3d Q_;

    // Measurement noise covariance (IMU noise)
    Eigen::Matrix3d R_;

    // Control input noise (encoder noise)
    Eigen::Matrix2d U_;

    // Constructor
    EKF() {
        state_.setZero();
        P_.setIdentity();
        Q_.setIdentity();
        R_.setIdentity();
        U_.setIdentity();
    }

    // Predict step based on encoder data
    void predict(double delta_x, double delta_y, double delta_theta, double dt) {
        // Prediction based on motion model
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 2) = -delta_y * sin(state_(2));  // Jacobian of motion model for x
        F(1, 2) = delta_x * cos(state_(2));   // Jacobian of motion model for y
        F(2, 2) = 1.0;                        // Jacobian of motion model for theta

        // Predicted state
        Eigen::Vector3d u(delta_x, delta_y, delta_theta);
        state_ += F * u;

        // Predicted covariance
        P_ = F * P_ * F.transpose() + Q_;

        // Apply control noise (if needed)
        P_ += U_;
    }

    // Correct step based on IMU measurement (angular velocity)
    void correct(double angular_velocity, double dt) {
        // Calculate the IMU measurement (IMU updates the angle)
        Eigen::Vector3d z(0.0, 0.0, angular_velocity * dt);

        // Measurement matrix
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        H(2, 2) = 1.0;  // IMU measures change in theta

        // Measurement residual
        Eigen::Vector3d y = z - H * state_;

        // Kalman gain
        Eigen::Matrix3d S = H * P_ * H.transpose() + R_;
        Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();

        // Update state estimate
        state_ = state_ + K * y;

        // Update covariance estimate
        P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
    }

    // Get the current state
    Eigen::Vector3d getState() const {
        return state_;
    }

    // Set process noise covariance matrix
    void setProcessNoise(const Eigen::Matrix3d& Q) {
        Q_ = Q;
    }

    // Set measurement noise covariance matrix
    void setMeasurementNoise(const Eigen::Matrix3d& R) {
        R_ = R;
    }

    // Set control input noise covariance matrix
    void setControlNoise(const Eigen::Matrix2d& U) {
        U_ = U;
    }
};