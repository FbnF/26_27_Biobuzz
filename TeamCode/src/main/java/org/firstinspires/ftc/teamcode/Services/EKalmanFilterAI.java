package org.firstinspires.ftc.teamcode.Services;

/**
 * Reusable Extended Kalman Filter (EKF) for a 4-wheeled mobile robot (differential/skid-steer).
 * Fuses high-rate goBILDA Pinpoint Odometry & BHI260AP IMU data with AprilTag measurements every 20 ms.
 */
public class EKalmanFilterAI {

    // State Vector: [x (inches), y (inches), theta (radians)]
    private double[] state = new double[3];

    // Error Covariance Matrix P (3x3) - uncertainty of our current state estimate
    private double[][] P = new double[3][3];

    // Process Noise Covariance Matrix Q (3x3) - uncertainty in the motion model (odometry + IMU)
    private double[][] Q = new double[3][3];

    // Measurement Noise Covariance Matrix R (3x3) - uncertainty in the AprilTag sensor measurements
    private double[][] R = new double[3][3];

    /**
     * Constructor to initialize the EKF with initial pose, distance sensor readings, and manual yaw.
     *
     * @param initialX     Initial X position (inches) derived from distance sensors
     * @param initialY     Initial Y position (inches) derived from distance sensors
     * @param initialTheta Initial heading angle (radians) from manual driver input / gyro setup
     */;
    public EKalmanFilterAI(double initialX, double initialY, double initialTheta) {
        state[0] = initialX;
        state[1] = initialY;
        state[2] = initialTheta;

        // Initialize Error Covariance P with initial starting uncertainties
        P[0][0] = 0.5;  // Initial X variance (inches^2) - tighter if distance sensors are precise
        P[1][1] = 0.5;  // Initial Y variance (inches^2)
        P[2][2] = 0.01; // Initial Theta variance (radians^2) - tight if manually aligned well

        // --- TUNING GUIDELINE FOR Q (Process Noise Matrix) ---
        // Q represents the cumulative error/drift introduced by wheel slip, uncalibrated encoder ticks,
        // and BHI260AP IMU gyro drift over a 20ms time window.
        // HOW TO TUNE:
        // 1. Drive the robot straight for 10 feet using only Pinpoint odometry, compare with a tape measure.
        // 2. Adjust Q[0][0] and Q[1][1] until the filter's estimated uncertainty matches your observed physical drift.
        // 3. Spin the robot 360 degrees multiple times to measure heading drift from the BHI260AP gyro, and set Q[2][2].
        Q[0][0] = 0.005; // Process noise variance for X per 20ms step
        Q[1][1] = 0.005; // Process noise variance for Y per 20ms step
        Q[2][2] = 0.0005;// Process noise variance for Heading (theta) per 20ms step

        // --- TUNING GUIDELINE FOR R (Measurement Noise Matrix) ---
        // R represents how much noise or jitter is present in your AprilTag pose estimations.
        // HOW TO TUNE:
        // 1. Position the robot stationary directly in front of an AprilTag at a typical scoring distance.
        // 2. Log 100 consecutive AprilTag X, Y, and Theta readings via telemetry.
        // 3. Compute the variance (standard deviation squared) of those logged values and assign them here.
        //    (Note: If tags are far away, dynamically scale R up because distant tags are noisier).
        R[0][0] = 0.1;  // AprilTag X measurement variance (inches^2)
        R[1][1] = 0.1;  // AprilTag Y measurement variance (inches^2)
        R[2][2] = 0.01; // AprilTag Heading measurement variance (radians^2)
    }

    /**
     * Main Reusable EKF Update Function. Called every 20 ms in the opmode loop.
     *
     * @param deltaDist     Change in distance traveled since last 20ms step from odometry (inches)
     * @param deltaTheta    Change in heading since last 20ms step from BHI260AP IMU (radians)
     * @param tagDetected   True if an AprilTag is currently visible in this frame, false otherwise
     * @param tagX          Observed absolute X coordinate from AprilTag (inches)
     * @param tagY          Observed absolute Y coordinate from AprilTag (inches)
     * @param tagTheta      Observed absolute heading from AprilTag (radians)
     * @return              Array containing the fused final pose: [x, y, theta]
     */
    public double[] update(double deltaDist, double deltaTheta, boolean tagDetected, double tagX, double tagY, double tagTheta) {

        // ==========================================
        // 1. PREDICT STEP (Executed every 20ms using Odometry & BHI260AP IMU)
        // ==========================================
        double theta = state[2];
        double avgTheta = theta + (deltaTheta / 2.0); // Midpoint heading approximation for better arc accuracy

        // Update state prediction using differential-drive kinematics
        state[0] += deltaDist * Math.cos(avgTheta);
        state[1] += deltaDist * Math.sin(avgTheta);
        state[2] = normalizeAngle(theta + deltaTheta);

        // Compute Jacobian of the motion model (F matrix)
        double[][] F = {
                {1.0, 0.0, -deltaDist * Math.sin(avgTheta)},
                {0.0, 1.0,  deltaDist * Math.cos(avgTheta)},
                {0.0, 0.0,  1.0}
        };

        // Propagate Error Covariance: P = F * P * F^T + Q
        double[][] FP = multiply3x3(F, P);
        double[][] FT = transpose3x3(F);
        double[][] FPFt = multiply3x3(FP, FT);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P[i][j] = FPFt[i][j] + Q[i][j];
            }
        }

        // ==========================================
        // 2. UPDATE / CORRECTION STEP (Executed when AprilTag data is available)
        // ==========================================
        if (tagDetected) {
            double[] Z = {tagX, tagY, normalizeAngle(tagTheta)};

            // Measurement matrix H (Identity because AprilTag gives direct X, Y, Theta)
            double[][] H = {
                    {1.0, 0.0, 0.0},
                    {0.0, 1.0, 0.0},
                    {0.0, 0.0, 1.0}
            };

            // Innovation / Residual: y = Z - H * state
            double[] y = new double[3];
            y[0] = Z[0] - state[0];
            y[1] = Z[1] - state[1];
            y[2] = normalizeAngle(Z[2] - state[2]);

            // Innovation Covariance: S = H * P * H^T + R (Simplifies to P + R since H is Identity)
            double[][] S = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    S[i][j] = P[i][j] + R[i][j];
                }
            }

            // Invert S matrix
            double[][] S_inv = invert3x3(S);

            // Kalman Gain: K = P * H^T * S_inv (Simplifies to P * S_inv)
            double[][] K = multiply3x3(P, S_inv);

            // Correct State: state = state + K * y
            for (int i = 0; i < 3; i++) {
                double correction = 0;
                for (int j = 0; j < 3; j++) {
                    correction += K[i][j] * y[j];
                }
                state[i] += correction;
            }
            state[2] = normalizeAngle(state[2]);

            // Correct Error Covariance: P = (I - K * H) * P
            double[][] I = {
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
            };
            double[][] I_minus_KH = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    I_minus_KH[i][j] = I[i][j] - K[i][j];
                }
            }

            double[][] newP = multiply3x3(I_minus_KH, P);
            for (int i = 0; i < 3; i++) {
                System.arraycopy(newP[i], 0, P[i], 0, 3);
            }
        }

        // Return final fused output: [X, Y, Theta]
        return new double[] { state[0], state[1], state[2] };
    }

    // ==================== HELPER MATH UTILITIES ====================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double[][] multiply3x3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    private double[][] transpose3x3(double[][] A) {
        double[][] At = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                At[i][j] = A[j][i];
            }
        }
        return At;
    }

    private double[][] invert3x3(double[][] A) {
        double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

        double invDet = 1.0 / det;
        double[][] inv = new double[3][3];

        inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
        inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
        inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
        inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
        inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
        inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;
        inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
        inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
        inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

        return inv;
    }
}