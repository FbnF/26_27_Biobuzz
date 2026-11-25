package org.firstinspires.ftc.teamcode.Testing;

/*
  Limelight Turning-Based Turret PID Auto-Align & Live Gamepad Tuner
  Based on Brogan M Pratt's AprilTag tracking architecture.
  Fallback turret auto-align option when KalmanFilter / MegaTag2 localization fails.

  PID Control Roles:
  - P (Proportional): Main corrective force proportional to current angle error.
  - I (Integral): Accumulates small steady-state error near target to overcome motor stiction.
  - D (Derivative): Damps overshoot and resists fast rate of change.

  Tuning Workflow:
  1. Start with kI = 0.0, kD = 0.0 and a small kP (e.g. 0.0001).
  2. Increase kP (D-Pad Right) until the turret snaps toward target tx.
  3. Increase kI (Right Bumper) if the turret stops slightly short of center (stiction).
  4. Increase kD (D-Pad Up) to damp any remaining overshoot or oscillation.
  5. Use B to cycle step sizes (0.1 down to 0.000001) for fine adjustments.
  6. Press A to reset gains or X for emergency stop.

  Controls (Gamepad 1):
  B Button             - Cycle tuning step size (0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001)
  D-Pad Left / Right   - Decrease / Increase kP
  Left / Right Bumper  - Decrease / Increase kI
  D-Pad Down / Up      - Decrease / Increase kD
  A Button             - Reset kP, kI, kD to defaults
  X Button             - Emergency stop turret
*/

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "LimelightTurningBasedTurret", group = "Testing")
public class LimelightTurret extends LinearOpMode {

    public static double KP = 0.0001;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double GOAL_ANGLE_DEG = 0.0;
    public static double ANGLE_TOLERANCE_DEG = 0.25;
    public static double INTEGRAL_ACTIVE_ZONE_DEG = 10.0; // Only accumulate integral within 10 degrees
    public static double MAX_POWER = 0.8;
    public static double MAX_I_POWER = 0.25; // Anti-windup max integral power contribution
    public static boolean INVERT_TURRET_DIRECTION = true;

    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    private double lastError = 0.0;
    private double integralSum = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private final double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    private int stepIndex = 3;

    private boolean prevB = false;
    private boolean prevA = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLB = false;
    private boolean prevRB = false;

    @Override
    public void runOpMode() {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {
            turretMotor = null;
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(3);
        } catch (Exception ignored) {
            limelight = null;
        }

        dashTelemetry.addLine("limelight turning turret ready");
        dashTelemetry.addData("turret motor", turretMotor != null ? "ok" : "missing");
        dashTelemetry.addData("limelight", limelight != null ? "ok" : "missing");
        dashTelemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        loopTimer.reset();

        try {
            while (opModeIsActive()) {

                Double targetTx = null;
                if (limelight != null) {
                    try {
                        LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            targetTx = result.getTx();
                        }
                    } catch (Exception ignored) {}
                }

                double dt = loopTimer.seconds();
                loopTimer.reset();

                double power = 0.0;
                double error = 0.0;

                if (targetTx == null) {
                    if (turretMotor != null) turretMotor.setPower(0.0);
                    lastError = 0.0;
                    integralSum = 0.0;
                } else {
                    error = GOAL_ANGLE_DEG - targetTx;

                    // Zero-crossing reset: clear integral if error sign changes to prevent overshoot
                    if (Math.copySign(1.0, error) != Math.copySign(1.0, lastError)) {
                        integralSum = 0.0;
                    }

                    // P-Term
                    double pTerm = error * KP;

                    // I-Term: Accumulate only within active zone near target to beat stiction
                    double absError = Math.abs(error);
                    if (absError > ANGLE_TOLERANCE_DEG && absError < INTEGRAL_ACTIVE_ZONE_DEG) {
                        integralSum += error * dt;
                    } else if (absError <= ANGLE_TOLERANCE_DEG) {
                        integralSum = 0.0;
                    }

                    double iTerm = Range.clip(integralSum * KI, -MAX_I_POWER, MAX_I_POWER);

                    // D-Term
                    double dTerm = 0.0;
                    if (dt > 0) {
                        dTerm = ((error - lastError) / dt) * KD;
                    }

                    if (absError <= ANGLE_TOLERANCE_DEG) {
                        power = 0.0;
                        integralSum = 0.0;
                    } else {
                        power = Range.clip(pTerm + iTerm + dTerm, -MAX_POWER, MAX_POWER);
                    }

                    double motorPower = INVERT_TURRET_DIRECTION ? -power : power;
                    if (turretMotor != null) turretMotor.setPower(motorPower);
                    lastError = error;
                }

                // gamepad tuning
                if (gamepad1.b && !prevB) {
                    stepIndex = (stepIndex + 1) % stepSizes.length;
                }
                prevB = gamepad1.b;

                if (gamepad1.a && !prevA) {
                    KP = 0.0001;
                    KI = 0.0;
                    KD = 0.0;
                    integralSum = 0.0;
                }
                prevA = gamepad1.a;

                if (gamepad1.x) {
                    if (turretMotor != null) turretMotor.setPower(0.0);
                    integralSum = 0.0;
                }

                double step = stepSizes[stepIndex];

                if (gamepad1.dpad_left && !prevLeft) KP = Math.max(0.0, KP - step);
                prevLeft = gamepad1.dpad_left;

                if (gamepad1.dpad_right && !prevRight) KP += step;
                prevRight = gamepad1.dpad_right;

                if (gamepad1.left_bumper && !prevLB) KI = Math.max(0.0, KI - step);
                prevLB = gamepad1.left_bumper;

                if (gamepad1.right_bumper && !prevRB) KI += step;
                prevRB = gamepad1.right_bumper;

                if (gamepad1.dpad_up && !prevUp) KD += step;
                prevUp = gamepad1.dpad_up;

                if (gamepad1.dpad_down && !prevDown) KD = Math.max(0.0, KD - step);
                prevDown = gamepad1.dpad_down;

                // ftc dashboard graph packet
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Target Tx (Deg)", targetTx != null ? targetTx : 0.0);
                packet.put("Error (Deg)", error);
                packet.put("Power", power);
                packet.put("kP", KP);
                packet.put("kI", KI);
                packet.put("kD", KD);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // driver station telemetry
                dashTelemetry.addLine("limelight turning turret tracking");
                dashTelemetry.addData("target tx", targetTx != null ? String.format("%.2f deg", targetTx) : "target lost");
                dashTelemetry.addData("error", "%.2f deg", error);
                dashTelemetry.addData("power", "%.3f", power);
                dashTelemetry.addData("kp (left/right)", "%.6f", KP);
                dashTelemetry.addData("ki (lb/rb)", "%.6f", KI);
                dashTelemetry.addData("kd (down/up)", "%.6f", KD);
                dashTelemetry.addData("step size (b)", "%.6f", step);
                dashTelemetry.update();
            }
        } finally {
            if (turretMotor != null) turretMotor.setPower(0.0);
        }
    }
}
