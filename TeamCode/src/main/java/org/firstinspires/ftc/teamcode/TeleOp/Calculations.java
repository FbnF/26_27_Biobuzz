package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;

// Math and geometry helpers for shooter distance and auto-align
public final class Calculations {

    private Calculations() {}

    // Distance from robot to target in inches
    public static double distance(Pose robotPose, double targetX, double targetY) {
        if (robotPose == null) return Double.NaN;
        double dx = targetX - robotPose.x();
        double dy = targetY - robotPose.y();
        return Math.hypot(dx, dy);
    }

    // Required angle in radians to face the target
    public static double angleToTarget(Pose robotPose, double targetX, double targetY) {
        if (robotPose == null) return 0.0;
        double dx = targetX - robotPose.x();
        double dy = targetY - robotPose.y();
        return Math.atan2(dy, dx);
    }

    // Wrap angle to [-PI, PI]
    public static double normalizeAngle(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    // Heading error between current robot heading and target angle
    public static double headingError(Pose robotPose, double targetX, double targetY) {
        if (robotPose == null) return 0.0;
        double targetAngle = angleToTarget(robotPose, targetX, targetY);
        return normalizeAngle(targetAngle - robotPose.heading());
    }

    // Checks if robot is inside the valid shooting cone
    public static boolean isInTriangularSector(Pose robotPose, double hiveX, double hiveY, double hiveFaceAngleRad, double maxSectorHalfAngleRad, double minDistIn, double maxDistIn) {
        if (robotPose == null) return false;

        double dist = distance(robotPose, hiveX, hiveY);
        if (Double.isNaN(dist) || dist < minDistIn || dist > maxDistIn) {
            return false;
        }

        // Angle from hive to robot vs hive face direction
        double angleFromHiveToRobot = Math.atan2(robotPose.y() - hiveY, robotPose.x() - hiveX);
        double angleDiffFromFace = Math.abs(normalizeAngle(angleFromHiveToRobot - hiveFaceAngleRad));

        return angleDiffFromFace <= maxSectorHalfAngleRad;
    }

    // Linear interpolation between near and far calibration points
    public static double interpolateLinear(double distance,
                                            double nearDist,
                                            double nearTPS,
                                            double farDist,
                                            double farTPS) {
        if (distance <= nearDist) return nearTPS;
        if (distance >= farDist) return farTPS;
        if (Math.abs(farDist - nearDist) < 1e-4) return nearTPS;

        double t = (distance - nearDist) / (farDist - nearDist);
        return nearTPS + t * (farTPS - nearTPS);
    }

    // Piecewise interpolation across distance/TPS lookup tables
    public static double interpolateTable(double distance, double[] distTable, double[] tpsTable) {
        if (distTable == null || tpsTable == null || distTable.length == 0 || tpsTable.length == 0) {
            return 0.0;
        }
        if (distTable.length != tpsTable.length) return tpsTable[0];
        if (distance <= distTable[0]) return tpsTable[0];

        int last = distTable.length - 1;
        if (distance >= distTable[last]) return tpsTable[last];

        int i = 0;
        while (i < last - 1 && distance > distTable[i + 1]) {
            i++;
        }

        return interpolateLinear(distance, distTable[i], tpsTable[i], distTable[i + 1], tpsTable[i + 1]);
    }
}
