package org.firstinspires.ftc.teamcode.Services;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.TinyCsvLoggerFlex;

import java.util.function.Supplier;

public class LoggingService {

    private static LoggingService instance = null;
    private TinyCsvLoggerFlex logger;
    private boolean enabled = true;

    public static synchronized LoggingService getInstance() {
        if (instance == null) {
            instance = new LoggingService();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap, Supplier<Pose> poseSupplier) {
        if (!enabled) return;

        DcMotorEx launchMotor = null;
        DcMotorEx intakeMotor = null;

        try { launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor"); } catch (Exception ignored) {}
        try { intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); } catch (Exception ignored) {}

        logger = TinyCsvLoggerFlex.create(
                hardwareMap,
                "teleop_new",
                TinyCsvLoggerFlex.motorEx("launch", launchMotor),
                TinyCsvLoggerFlex.motorEx("intake", intakeMotor),
                TinyCsvLoggerFlex.pedroPose("pose", poseSupplier)
        );
    }

    public void record() {
        if (enabled && logger != null) {
            logger.record("run");
        }
    }

    public void stop() {
        if (logger != null) {
            logger.close();
            logger = null;
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }
}
