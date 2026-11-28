package org.firstinspires.ftc.teamcode.Tuning;

/*
  Live Flywheel PIDF Tuner OpMode
  Based on Brogan M. Pratt's FTC Flywheel PIDF Tuning tutorial.

  HOW TO USE THIS TUNER ON YOUR DRIVER STATION:
  1. Mapped to "launchMotor" (DcMotorEx) in REVERSE direction.
  2. Target velocities set to highVelocity = 3200.0 TPS and lowVelocity = 2700.0 TPS.
  3. Start OpMode with P = 0, I = 0, F = 0.
  4. TUNE F (FEEDFORWARD) FIRST:
     - F is baseline power prediction to hold target speed.
     - Press D-Pad Right/Left to increase/decrease F (press 'B' to change step size).
     - Adjust F until Actual Velocity matches highVelocity.
  5. TUNE P (PROPORTIONAL) SECOND:
     - P controls recovery speed when RPM drops after firing a ball.
     - Press 'Y' to toggle between highVelocity (3200) and lowVelocity (2700).
     - Press D-Pad Up/Down to increase/decrease P until actual velocity recovers instantly.
  6. TUNE I (INTEGRAL) OPTIONAL:
     - Press Right/Left Bumper to adjust I if steady-state error remains near target speed.
  7. APPLY:
     - Record tuned P, I, F numbers and apply them to PIDFManualTeleop.

  Controls (Gamepad 1):
  Y Button              - Toggle between highVelocity (3200) and lowVelocity (2700)
  B Button              - Cycle step size (10.0, 1.0, 0.1, 0.01, 0.001)
  D-Pad Left / Right    - Decrease / Increase Feedforward (F)
  D-Pad Up / Down       - Increase / Decrease Proportional (P)
  Right / Left Bumper   - Increase / Decrease Integral (I)
  A Button              - Reset P, I, F gains to 0.0
  X Button              - Toggle Flywheel Motor ON / OFF
*/

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PIDFFlywheelTuning", group = "Tuning")
public class PIDFFlywheelTuning extends LinearOpMode {

    public static double HIGH_VELOCITY = 3200.0;
    public static double LOW_VELOCITY = 2700.0;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;

    private DcMotorEx launchMotor;

    private double curTargetVelocity = HIGH_VELOCITY;
    private boolean isMotorOn = true;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1; // Default step = 1.0

    private boolean prevY = false;
    private boolean prevB = false;
    private boolean prevA = false;
    private boolean prevX = false;
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
            launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {
            launchMotor = null;
        }

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        curTargetVelocity = HIGH_VELOCITY;

        dashTelemetry.addLine("pidf flywheel tuner ready");
        dashTelemetry.addData("launch motor", launchMotor != null ? "ok" : "missing");
        dashTelemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {

                // Gamepad 1 Controls
                if (gamepad1.y && !prevY) {
                    curTargetVelocity = (curTargetVelocity == HIGH_VELOCITY) ? LOW_VELOCITY : HIGH_VELOCITY;
                }
                prevY = gamepad1.y;

                if (gamepad1.b && !prevB) {
                    stepIndex = (stepIndex + 1) % stepSizes.length;
                }
                prevB = gamepad1.b;

                if (gamepad1.a && !prevA) {
                    P = 0.0;
                    I = 0.0;
                    F = 0.0;
                }
                prevA = gamepad1.a;

                if (gamepad1.x && !prevX) {
                    isMotorOn = !isMotorOn;
                }
                prevX = gamepad1.x;

                double step = stepSizes[stepIndex];

                if (gamepad1.dpad_left && !prevLeft) {
                    F = Math.max(0.0, F - step);
                }
                prevLeft = gamepad1.dpad_left;

                if (gamepad1.dpad_right && !prevRight) {
                    F += step;
                }
                prevRight = gamepad1.dpad_right;

                if (gamepad1.dpad_up && !prevUp) {
                    P += step;
                }
                prevUp = gamepad1.dpad_up;

                if (gamepad1.dpad_down && !prevDown) {
                    P = Math.max(0.0, P - step);
                }
                prevDown = gamepad1.dpad_down;

                if (gamepad1.right_bumper && !prevRB) {
                    I += step;
                }
                prevRB = gamepad1.right_bumper;

                if (gamepad1.left_bumper && !prevLB) {
                    I = Math.max(0.0, I - step);
                }
                prevLB = gamepad1.left_bumper;

                double activeTarget = isMotorOn ? curTargetVelocity : 0.0;
                double actualVelocity = (launchMotor != null) ? launchMotor.getVelocity() : 0.0;
                double motorPower = (launchMotor != null) ? launchMotor.getPower() : 0.0;
                double error = activeTarget - actualVelocity;

                if (launchMotor != null) {
                    PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
                    launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                    launchMotor.setVelocity(activeTarget);
                }

                // FTC Dashboard graph packet
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Target Velocity (TPS)", activeTarget);
                packet.put("Actual Velocity (TPS)", actualVelocity);
                packet.put("Error (TPS)", error);
                packet.put("Tuned P", P);
                packet.put("Tuned I", I);
                packet.put("Tuned F", F);
                packet.put("Motor Power", motorPower * 1000.0);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // Driver Station telemetry
                dashTelemetry.addLine("PIDF Flywheel Tuner Running");
                dashTelemetry.addData("Motor Status", isMotorOn ? "ON" : "STOPPED");
                dashTelemetry.addData("Target Velocity", "%.0f tps", activeTarget);
                dashTelemetry.addData("Actual Velocity", "%.0f tps", actualVelocity);
                dashTelemetry.addData("Velocity Error", "%.0f tps", error);
                dashTelemetry.addData("Tuned P (dpad up/down)", "%.5f", P);
                dashTelemetry.addData("Tuned I (rb/lb)", "%.5f", I);
                dashTelemetry.addData("Tuned F (dpad right/left)", "%.5f", F);
                dashTelemetry.addData("Step Size (b)", "%.3f", step);
                dashTelemetry.update();
            }
        } finally {
            if (launchMotor != null) {
                launchMotor.setVelocity(0.0);
                launchMotor.setPower(0.0);
            }
        }
    }
}
