#pragma once

class SimpleKalmanFilter {
public:
    SimpleKalmanFilter(double process_noise, double measurement_noise, double estimated_error, double initial_value) {
        Q = process_noise;
        R = measurement_noise;
        P = estimated_error;
        X = initial_value;
    }

    double update(double measurement) {
        // Prediction update
        P = P + Q;

        // Measurement update
        K = P / (P + R);
        X = X + K * (measurement - X);
        P = (1 - K) * P;

        return X;
    }

private:
    double Q; // Process noise covariance
    double R; // Measurement noise covariance
    double P; // Estimation error covariance
    double K; // Kalman gain
    double X; // Value
};
