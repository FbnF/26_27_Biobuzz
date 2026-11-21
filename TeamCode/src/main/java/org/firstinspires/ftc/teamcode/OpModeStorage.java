package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class OpModeStorage {

    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public static boolean isPoseFresh = false;
    public static long autoSaveTimestampMs = 0L;

    public static synchronized void saveAutoEndPose(Pose pose) {
        if (pose != null) {
            autonomousEndPose = pose;
            isPoseFresh = true;
            autoSaveTimestampMs = System.currentTimeMillis();
        }
    }

    public static synchronized Pose getPoseForTeleOp(Pose fallbackPose) {
        long now = System.currentTimeMillis();
        long elapsedMs = now - autoSaveTimestampMs;

        // Fresh if saved within 5 minutes (300,000 ms) and isPoseFresh is true
        if (isPoseFresh && elapsedMs >= 0 && elapsedMs <= 300_000L) {
            isPoseFresh = false; // Consume so it cannot be reused by another run
            return autonomousEndPose;
        }

        return fallbackPose != null ? fallbackPose : new Pose(0, 0, 0);
    }
}
