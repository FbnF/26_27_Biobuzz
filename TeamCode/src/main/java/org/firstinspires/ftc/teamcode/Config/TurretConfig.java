package org.firstinspires.ftc.teamcode.Config;
import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvCamera;

@Config
public final class TurretConfig {
    //Angle in radians outside of what we can turn to.
    public static double max = 5.14;
    public static double min = 0;
    public static final double radius = 2.23607;

    //Bang-bang turret controller tuning.
    //TICKS_PER_RADIAN must be measured on the robot from the turret encoder + gearing.
    public static double TICKS_PER_RADIAN = 500.0;
    public static double BANG_BANG_POWER = 0.5;
    public static double BANG_BANG_DEADBAND = 20.0;
    public static double BANG_BANG_HYSTERESIS = 30.0;
    public static double gearRatio = 10/38;
}
