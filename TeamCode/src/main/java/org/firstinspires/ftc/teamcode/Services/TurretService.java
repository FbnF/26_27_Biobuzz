package org.firstinspires.ftc.teamcode.Services;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.TurretConfig;

/**
 * Turret control service.
 *
 * Uses the PID controller from LimelightTurret: P for the main correction,
 * I (only near the target) to overcome stiction, and D to damp overshoot.
 * The error is derived from the turret encoder angle instead of Limelight tx.
 */
@Config
public class TurretService {

    // PID tuning (ported from LimelightTurret)
    public static double KP = 0.0001;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double ANGLE_TOLERANCE_DEG = 0.25;
    public static double INTEGRAL_ACTIVE_ZONE_DEG = 10.0; // Only accumulate integral within 10 degrees
    public static double MAX_POWER = 0.8;
    public static double MAX_I_POWER = 0.25; // Anti-windup max integral power contribution

    // Encoder positive power already turns toward positive angles. Flip if the gearing reverses it.
    public static boolean INVERT_TURRET_DIRECTION = false;

    private DcMotorEx turretMotor;

    private int targetTicks = 0;
    private double targetAngle = 0.0;

    private double lastError = 0.0;
    private double integralSum = 0.0;
    private boolean pidInitialized = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private double absPosition = 0;
    private boolean turretInitialized = false;

    public void initTurret(DcMotorEx motor) {
        turretMotor = motor;
        if (turretMotor != null) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setPower(0);
        }

        targetTicks = 0;
        targetAngle = 0.0;
        resetPid();
        pidTimer.reset();
        turretInitialized = true;
    }

    /**
     * Kept for existing callers. The turret is both driven and measured with the encoder motor.
     */
    public void initTurret(DcMotorEx motor, DcMotorEx unusedServoMotor) {
        initTurret(motor);
    }

    public double getNewPosition(double positionChange) {
        absPosition = absPosition - positionChange;
        return absPosition;
    }

    public void setTargetAngle(double radians) {
        double clamped = Math.max(TurretConfig.min, Math.min(TurretConfig.max, radians));

        // A new target starts a fresh PID run so the derivative doesn't kick and old integral isn't reused.
        if (Math.abs(clamped - targetAngle) > 1e-9) {
            resetPid();
        }
        targetAngle = clamped;
        setTargetTicks((int) Math.round(clamped * TurretConfig.TICKS_PER_RADIAN));
    }

    public void setTargetTicks(int ticks) {
        int minTicks = (int) Math.round(TurretConfig.min * TurretConfig.TICKS_PER_RADIAN);
        int maxTicks = (int) Math.round(TurretConfig.max * TurretConfig.TICKS_PER_RADIAN);
        if (ticks < minTicks) {
            targetTicks = minTicks;
        } else if (ticks > maxTicks) {
            targetTicks = maxTicks;
        } else {
            targetTicks = ticks;
        }
    }


    public void updateTurret() {
        if (turretMotor == null) {
            return;
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();

        double currentAngle = turretMotor.getCurrentPosition() / TurretConfig.TICKS_PER_RADIAN;
        double error = Math.toDegrees(targetAngle - currentAngle);
        double absError = Math.abs(error);

        double power;
        if (absError <= ANGLE_TOLERANCE_DEG) {
            power = 0.0;
            integralSum = 0.0;
            lastError = error;
        } else {            if (pidInitialized && Math.copySign(1.0, error) != Math.copySign(1.0, lastError)) {
            integralSum = 0.0;
        }

            double pTerm = error * KP;

            if (absError > ANGLE_TOLERANCE_DEG && absError < INTEGRAL_ACTIVE_ZONE_DEG  && dt <= 0) {
                integralSum += error * dt;
            }
            double iTerm = Range.clip(integralSum * KI, -MAX_I_POWER, MAX_I_POWER);

            double dTerm = 0.0;
            if (pidInitialized && dt > 0) {
                dTerm = ((error - lastError) / dt) * KD;
            }

            power = Range.clip(pTerm + iTerm + dTerm, -MAX_POWER, MAX_POWER);

        }

        turretMotor.setPower(INVERT_TURRET_DIRECTION ? -power : power);
        lastError = error;
        pidInitialized = true;
    }

    private void resetPid() {
        integralSum = 0.0;
        lastError = 0.0;
        pidInitialized = false;
    }

    public void stopTurret() {
        if (turretMotor != null) {
            turretMotor.setPower(0);
        }
        resetPid();
    }

    public boolean atTarget() {
        if (turretMotor == null) {
            return false;
        }
        double currentAngle = turretMotor.getCurrentPosition() / TurretConfig.TICKS_PER_RADIAN;
        return Math.abs(Math.toDegrees(targetAngle - currentAngle)) <= ANGLE_TOLERANCE_DEG;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public int getCurrentTicks() {
        return turretMotor != null ? turretMotor.getCurrentPosition() : 0;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isTurretInitialized() {
        return turretInitialized;
    }

    public double GetRotationAngle(double targetX, double targetY) {
        double itargetX = -targetX;
        if (itargetX == 0.0 && targetY == 0.0) {
            return TurretConfig.min;
        }
        double angleOfHive = Math.atan2(itargetX, targetY);
        if (angleOfHive < TurretConfig.min) {
            return TurretConfig.min;
        }
        if (angleOfHive > TurretConfig.max) {
            return TurretConfig.max;
        }
        return angleOfHive;
    }

}
