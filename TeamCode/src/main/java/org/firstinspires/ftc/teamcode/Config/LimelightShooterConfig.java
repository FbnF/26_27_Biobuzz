package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class LimelightShooterConfig {

    private LimelightShooterConfig() {}

    public static double MIN_RANGE_IN = 10.0;
    public static double TPS_TOL = 50.0;

    // ---------------- Tag Group A Tables (IDs: 38..41 [Blue], 30..33 [Red]) ----------------
    public static double[] DIST_IN_GROUP_A = new double[] {
            70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0
    };
    public static double[] TPS_AT_DIST_GROUP_A = new double[] {
            1195.0, 1165.0, 1185.0, 1190.0, 1195.0, 1205.0, 1205.0
    };
    public static double[] TURN_ANGLE_GROUP_A = new double[] {
            0.0, 0.2, -0.2, 0.3, -0.3, 0.5, -0.5
    };

    // ---------------- Tag Group B Tables (IDs: 42..45 [Blue], 34..37 [Red]) ----------------
    public static double[] DIST_IN_GROUP_B = new double[] {
            70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0
    };
    public static double[] TPS_AT_DIST_GROUP_B = new double[] {
            1195.0, 1165.0, 1185.0, 1190.0, 1195.0, 1205.0, 1205.0
    };
    public static double[] TURN_ANGLE_GROUP_B = new double[] {
            0.0, 0.2, -0.2, 0.3, -0.3, 0.5, -0.5
    };

    // Legacy Blue / Red aliases pointing to Group A / Group B
    public static double[] DIST_IN_BLUE = DIST_IN_GROUP_A;
    public static double[] TPS_AT_DIST_BLUE = TPS_AT_DIST_GROUP_A;
    public static double[] TURN_ANGLE_BLUE = TURN_ANGLE_GROUP_A;

    public static double[] DIST_IN_RED = DIST_IN_GROUP_B;
    public static double[] TPS_AT_DIST_RED = TPS_AT_DIST_GROUP_B;
    public static double[] TURN_ANGLE_RED = TURN_ANGLE_GROUP_B;

    public static boolean isGroupA(int tagId) {
        return (tagId >= 38 && tagId <= 41) || (tagId >= 30 && tagId <= 33);
    }

    public static boolean isGroupB(int tagId) {
        return (tagId >= 42 && tagId <= 45) || (tagId >= 34 && tagId <= 37);
    }

    public static double interpolate(double distance, double[] distTable, double[] valTable) {
        if (distTable == null || valTable == null || distTable.length < 2) return 0.0;
        if (distance <= distTable[0]) return valTable[0];
        int last = distTable.length - 1;
        if (distance >= distTable[last]) return valTable[last];

        int i = 0;
        while (i < last - 1 && distance > distTable[i + 1]) {
            i++;
        }

        double x0 = distTable[i];
        double x1 = distTable[i + 1];
        double y0 = valTable[i];
        double y1 = valTable[i + 1];

        return y0 + (distance - x0) * (y1 - y0) / (x1 - x0);
    }
}
