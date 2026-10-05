package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

// Service for odometry flywheel speed calculation and targeting
@Config
public class OdometryShooterService {

    private static OdometryShooterService instance = null;

    // Target hive coordinates(assumption)
    public static double BLUE_HIVE_X = 141.5;
    public static double BLUE_HIVE_Y = 141.5;
    public static double RED_HIVE_X = 0.0;
    public static double RED_HIVE_Y = 141.5;

    // Hive opening directions in radians(assumption)
    public static double BLUE_HIVE_FACE_RAD = Math.toRadians(225.0);
    public static double RED_HIVE_FACE_RAD = Math.toRadians(315.0);

    // Shooting cone and distance cutoffs(assumption)
    public static double MAX_SECTOR_HALF_ANGLE_DEG = 45.0;
    public static double MIN_SHOOT_DIST_IN = 20.0;
    public static double MAX_SHOOT_DIST_IN = 120.0;

    // Near and far calibration endpoints(assumption)
    public static double CALIB_NEAR_DIST_IN = 30.0;
    public static double CALIB_NEAR_TPS = 1200.0;
    public static double CALIB_FAR_DIST_IN = 100.0;
    public static double CALIB_FAR_TPS = 1650.0;

    // Active state
    private boolean isBlueAlliance = true;
    private boolean isGoingToShoot = false;

    public static OdometryShooterService getInstance() {
        if (instance == null) {
            instance = new OdometryShooterService();
        }
        return instance;
    }

    public void setBlueAlliance(boolean isBlue) {
        this.isBlueAlliance = isBlue;
    }

    public boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    public void setGoingToShoot(boolean active) {
        this.isGoingToShoot = active;
    }

    public boolean isGoingToShoot() {
        return isGoingToShoot;
    }

    public void toggleGoingToShoot() {
        this.isGoingToShoot = !this.isGoingToShoot;
    }

    // Evaluates current pose against hive geometry and returns shooting state
    public ShootingState evaluateShooting(Pose robotPose) {
        if (!isGoingToShoot) {
            return ShootingState.disabled("GOING_TO_SHOOT_DISABLED");
        }

        if (robotPose == null) {
            return ShootingState.disabled("NULL_ROBOT_POSE");
        }

        double hiveX = isBlueAlliance ? BLUE_HIVE_X : RED_HIVE_X;
        double hiveY = isBlueAlliance ? BLUE_HIVE_Y : RED_HIVE_Y;
        double faceAngleRad = isBlueAlliance ? BLUE_HIVE_FACE_RAD : RED_HIVE_FACE_RAD;

        double dist = Calculations.distance(robotPose, hiveX, hiveY);
        double targetHeading = Calculations.angleToTarget(robotPose, hiveX, hiveY);
        double headingErr = Calculations.headingError(robotPose, hiveX, hiveY);

        boolean validSector = Calculations.isInTriangularSector(robotPose, hiveX, hiveY, faceAngleRad, Math.toRadians(MAX_SECTOR_HALF_ANGLE_DEG), MIN_SHOOT_DIST_IN, MAX_SHOOT_DIST_IN
        );

        if (!validSector) {
            String reason = dist < MIN_SHOOT_DIST_IN ? "TOO_CLOSE" :
                    (dist > MAX_SHOOT_DIST_IN ? "TOO_FAR" : "OUT_OF_SECTOR");
            return new ShootingState(false, dist, targetHeading, headingErr, 0.0, reason);
        }

        double calculatedTPS = Calculations.interpolateLinear(
                dist,
                CALIB_NEAR_DIST_IN,
                CALIB_NEAR_TPS,
                CALIB_FAR_DIST_IN,
                CALIB_FAR_TPS
        );

        return new ShootingState(true, dist, targetHeading, headingErr, calculatedTPS, "VALID");
    }
}
