package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class ShooterConfig {
    private ShooterConfig() {} // static-only

    // Existing values
    private static double[] distanceListRed = {75,58,52,51};
    private static double[] distanceListBlue = {75,58,52,51};
    private static double[] TPSListBlue = {0,0,0,0,0};
    private static double[] TPSListRed = {0,0,0,0,0};

    private static double TPSBlueOffset = 40;
    private static double TPSRedOffset = 40;

    // ---------------- Controls / Limits ----------------
    public static double MIN_RANGE_IN = 10.0;    // ignore ranges under this (inches)
    public static double TPS_MIN_AUTO = 800.0;   // below this, AUTO stays off
    public static double TPS_TOL = 50.0;

    public static double TPS_MAX_MECH = 2800.0;  // absolute safety limit
    public static double TPS_MAX_AUTO = 2400.0;  // clamp used by AUTO

    // ---------------- Mode Switches ----------------
    public static boolean USE_TABLE = true;
    public static boolean USE_FEED_TABLE = true;

    // Default powers for manual/auto modes
    public static double FEED_DEFAULT = -0.9;
    public static double SIDE_POWER = -0.9;
    public static double SIDE_POWER_LONG = -0.2;
    public static double SIDE_DEFAULT = 1.0;
    public static double INTAKE_POWER = 0.90;

    // Timing
    public static double START_WAIT_TIME = 2.0;
    public static double WAIT_TIME = 10.5;

    // Tuning knobs for physics mode
    public static double TPS_SCALE = 1.0;
    public static double TPS_OFFSET = 0.0;

    // No-shot zone
    public static double NO_SHOT_UNDER_IN = 0.0;

    // Smoothing (0 = off, 1 = heavy)
    public static double DIST_SMOOTH_ALPHA = 0.15;

    // Physics model params
    public static double G = 9.81;
    public static double LAUNCH_DEG = 46.0;
    public static double TARGET_H_M = 0.984;
    public static double SHOOTER_H_M = 0.248;
    public static double WHEEL_RADIUS_M = 0.048;
    public static double EFFICIENCY = 0.30;
    public static double TICKS_PER_REV = 28.0;

    // Handoff sensor
    public static double HANDOFF_DISTANCE_MM = 107.0;

    // Auto Align Config
    public static boolean AUTO_ALIGN_ENABLED = true;
    public static double ALIGN_KP = 0.02;
    public static double ALIGN_KD = 0.00045;
    public static double ALIGN_MAX_TURN = 0.7;
    public static double ALIGN_MIN_TURN = 0.08;
    public static double ALIGN_ERR_DEADBAND_DEG = 0.25;
    public static double ALIGN_MAX_STALE_MS = 100;

    // ---------------- Tag Group A Tables (IDs: 38..41 [Blue], 30..33 [Red]) ----------------
    public static double[] DIST_IN_GROUP_A = new double[] {
            70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0
    };
    public static double[] TPS_AT_DIST_GROUP_A = new double[] {
            1195.0, 1165.0, 1185.0, 1190.0, 1195.0, 1205.0, 1205.0
    };
    public static double[] MIN_ANGLE_GROUP_A = new double[] {
            -0.5, 0.2, -0.5, -1.2, 0.2, -0.5, 0.2
    };
    public static double[] MAX_ANGLE_GROUP_A = new double[] {
            0.5, 1.2, 0.5, -0.2, 1.2, 0.5, 1.2
    };
    public static double[] FEED_POWER_GROUP_A = new double[] {
            -0.9, -0.9, -0.9, -0.9, -0.9, -0.9, -0.9
    };
    public static double[] SIDE_POWER_GROUP_A = new double[] {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    };

    // ---------------- Tag Group B Tables (IDs: 42..45 [Blue], 34..37 [Red]) ----------------
    public static double[] DIST_IN_GROUP_B = new double[] {
            70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0
    };
    public static double[] TPS_AT_DIST_GROUP_B = new double[] {
            1195.0, 1165.0, 1185.0, 1190.0, 1195.0, 1205.0, 1205.0
    };
    public static double[] MIN_ANGLE_GROUP_B = new double[] {
            -0.5, 0.2, -1.2, 0.2, -1.2, 0.2, 0.2
    };
    public static double[] MAX_ANGLE_GROUP_B = new double[] {
            0.5, 1.2, -0.2, 1.2, -0.2, 1.2, 1.2
    };
    public static double[] FEED_POWER_GROUP_B = new double[] {
            -0.9, -0.9, -0.9, -0.9, -0.9, -0.9, -0.9
    };
    public static double[] SIDE_POWER_GROUP_B = new double[] {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    };

    // Legacy Blue / Red table references
    public static double[] DIST_IN_BLUE = DIST_IN_GROUP_A;
    public static double[] TPS_AT_DIST_BLUE = TPS_AT_DIST_GROUP_A;
    public static double[] MIN_ANGLE_BLUE = MIN_ANGLE_GROUP_A;
    public static double[] MAX_ANGLE_BLUE = MAX_ANGLE_GROUP_A;
    public static double[] FEED_POWER_BLUE = FEED_POWER_GROUP_A;
    public static double[] SIDE_POWER_BLUE = SIDE_POWER_GROUP_A;

    public static double[] DIST_IN_RED = DIST_IN_GROUP_B;
    public static double[] TPS_AT_DIST_RED = TPS_AT_DIST_GROUP_B;
    public static double[] MIN_ANGLE_RED = MIN_ANGLE_GROUP_B;
    public static double[] MAX_ANGLE_RED = MAX_ANGLE_GROUP_B;
    public static double[] FEED_POWER_RED = FEED_POWER_GROUP_B;
    public static double[] SIDE_POWER_RED = SIDE_POWER_GROUP_B;

    // Helper method to determine if an AprilTag ID is in Group A (IDs: 38..40, 30..32)
    public static boolean isGroupA(int tagId) {
        return (tagId >= 38 && tagId <= 40) || (tagId >= 30 && tagId <= 32);
    }

    // Helper method to determine if an AprilTag ID is in Group B (IDs: 42..44, 34..36)
    public static boolean isGroupB(int tagId) {
        return (tagId >= 42 && tagId <= 44) || (tagId >= 34 && tagId <= 36);
    }

    // Constant Velocities for Auto
    public static double SHOOTER_VEL_LONG = 1550.0;
    public static double SHOOTER_VEL_SHORT = 1175.0;

    // Teleops
    public static double DISTANCE_TO_SWITCH_TO_FAR = 70;

    public static double RED_ALIGN_OFFSET = 1.0;
    public static double RED_ALIGN_FAR_OFFSET = 1.0;

    public static double BLUE_ALIGN_OFFSET = 1.0;
    public static double BLUE_ALIGN_FAR_OFFSET = 1.0;

    // Front Servo Positional Torque Positions
    public static double FRONT_SERVO_INIT_POS = 0.06;
    public static double FRONT_SERVO_FLOWER_POS = 0.35;

    // Intake Helper Powers
    public static double INTAKE_HELPER_INTAKE_POWER = -1.0;
    public static double INTAKE_HELPER_REVERSE_POWER = 1.0;
}
