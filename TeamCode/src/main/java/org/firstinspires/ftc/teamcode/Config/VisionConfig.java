package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class VisionConfig {
    private VisionConfig() {}

    // Camera / pipeline
    public static boolean USE_WEBCAM = true;
    public static String WEBCAM_NAME = "Webcam 1";

    // What distance to use for shooter math
    // true  -> use straight-line range (inches)
    // false -> use ftcPose.y (inches), if that better matches your geometry
    public static boolean USE_RANGE = true;

    // If > 0, prefer this tag ID when multiple are visible; else pick closest
    public static int PREFERRED_ID = 0;

    // Simple exponential smoothing on distance (0 = off, 1 = no smoothing)
    public static double SMOOTH_ALPHA = 0.5;

    // Debug drawing on the preview
    public static boolean DRAW_AXES = true;
    public static boolean DRAW_OUTLINE = true;
    public static boolean DRAW_ID = true;
}