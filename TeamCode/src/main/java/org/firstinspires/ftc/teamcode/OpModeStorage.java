package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;


 //Stores the final robot Pose from Autonomous so TeleOp can load it at startup.

public class OpModeStorage {
    // Default pose if Auto was not run beforehand (Field center, facing 90 deg / +Y)
    public static Pose autonomousEndPose = new Pose(72.0, 12.0, Math.toRadians(90.0));
}
