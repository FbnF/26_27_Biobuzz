package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(name = "PIDFManualTeleop", group = "TeleOp")
public class PIDFManualTeleop extends LinearOpMode {

    private Follower follower;

    private DcMotorEx launchMotor;
    private DcMotorEx turretMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx intakeMotor;
    private Servo gateServo;
    private CRServo contServo;

    // Tuned PIDF Coefficients (enter final numbers from PIDFFlywheelTuning here)
    public static double TUNED_P = 500.0;
    public static double TUNED_I = 0.0;
    public static double TUNED_D = 0.0;
    public static double TUNED_F = 4.0;

    public static double TPS_TOL = 25.0;
    public static double TPS_STEP = 25.0; // 25 TPS per D-Pad Up/Down press

    private boolean shooting = false;
    private double gatePos = 0.11;
    private double intakeTPS = 0.0;
    private double shooterTPS = 3200.0;
    private double curTransferTps = 700.0;
    private double turretPower = 0.0;

    private boolean lastRightTrigger2 = false;
    private boolean lastLeftTrigger2 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastDpadRight2 = false;
    private boolean lastDpadLeft2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastY2 = false;
    private boolean lastX2 = false;

    @Override
    public void runOpMode() {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);

        try {
            launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {
            launchMotor = null;
        }

        try { turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"); } catch (Exception ignored) {}
        try { transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor"); } catch (Exception ignored) {}
        try { intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); } catch (Exception ignored) {}
        try { gateServo = hardwareMap.get(Servo.class, "gateServo"); } catch (Exception ignored) {}
        try { contServo = hardwareMap.get(CRServo.class, "contServo"); } catch (Exception ignored) {}

        if (contServo != null) {
            contServo.setDirection(CRServo.Direction.REVERSE);
            contServo.setPower(0);
        }

        if (gateServo != null) {
            gateServo.setPosition(gatePos);
        }

        dashTelemetry.addLine("PIDF Manual TeleOp Ready");
        dashTelemetry.addData("Launch Motor", launchMotor != null ? "OK" : "MISSING");
        dashTelemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {
                follower.update();

                double axial = -gamepad1.right_stick_y;
                double lateral = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;
                follower.manual(axial, lateral, turn);

                intakeLogic();
                tpsAdjustments();
                shooterLogic();
                settingPowers();
                postData(dashTelemetry);

                updateButtonStates();
            }
        } finally {
            if (launchMotor != null) {
                launchMotor.setVelocity(0.0);
                launchMotor.setPower(0.0);
            }
            if (intakeMotor != null) intakeMotor.setPower(0.0);
            if (transferMotor != null) transferMotor.setPower(0.0);
            if (contServo != null) contServo.setPower(0.0);
            if (turretMotor != null) turretMotor.setPower(0.0);
        }
    }

    private void updateButtonStates() {
        lastRightTrigger2 = gamepad2.right_trigger > 0.5;
        lastLeftTrigger2 = gamepad2.left_trigger > 0.5;
        lastLeftBumper2 = gamepad2.left_bumper;
        lastDpadRight2 = gamepad2.dpad_right;
        lastDpadLeft2 = gamepad2.dpad_left;
        lastDpadUp2 = gamepad2.dpad_up;
        lastDpadDown2 = gamepad2.dpad_down;
        lastY2 = gamepad2.y;
        lastX2 = gamepad2.x;
    }

    private void intakeLogic() {
        boolean rightTriggerPressed = (gamepad2.right_trigger > 0.5) && !lastRightTrigger2;
        boolean leftTriggerPressed = (gamepad2.left_trigger > 0.5) && !lastLeftTrigger2;
        boolean leftBumperPressed = gamepad2.left_bumper && !lastLeftBumper2;

        if (rightTriggerPressed && !shooting) {
            if (transferMotor != null) transferMotor.setPower(0.7);
            intakeTPS = 0.9;
        }
        if (leftTriggerPressed && !shooting) {
            if (transferMotor != null) transferMotor.setPower(0.7);
            intakeTPS = 0.0;
        }
        if (leftBumperPressed && !shooting) {
            if (transferMotor != null) transferMotor.setPower(-0.7);
            intakeTPS = -0.9;
        }
    }

    private void tpsAdjustments() {
        boolean dpadRightPressed = gamepad2.dpad_right && !lastDpadRight2;
        boolean dpadLeftPressed = gamepad2.dpad_left && !lastDpadLeft2;
        boolean dpadUpPressed = gamepad2.dpad_up && !lastDpadUp2;
        boolean dpadDownPressed = gamepad2.dpad_down && !lastDpadDown2;

        if (dpadRightPressed && !shooting) curTransferTps += 15.0;
        if (dpadLeftPressed && !shooting) curTransferTps -= 15.0;
        if (dpadUpPressed) shooterTPS += TPS_STEP; // Increments by 25 TPS
        if (dpadDownPressed) shooterTPS = Math.max(0.0, shooterTPS - TPS_STEP);
    }

    private void shooterLogic() {
        boolean yPressed = gamepad2.y && !lastY2;
        boolean xPressed = gamepad2.x && !lastX2;

        if (yPressed) {
            shooting = true;
            if (transferMotor != null) transferMotor.setPower(0.75);
            intakeTPS = 0.9;
            gatePos = 0.35;
            if (contServo != null) contServo.setPower(1.0);
        }
        if (xPressed) {
            shooting = false;
            if (transferMotor != null) transferMotor.setPower(1.0);
            intakeTPS = 0.9;
            gatePos = 0.11;
            if (contServo != null) contServo.setPower(0.0);
        }
        turretPower = gamepad2.right_stick_x;
    }

    private void settingPowers() {
        if (gateServo != null) gateServo.setPosition(gatePos);
        if (intakeMotor != null) intakeMotor.setPower(intakeTPS);
        if (turretMotor != null) turretMotor.setPower(turretPower);

        if (launchMotor != null) {
            if (shooterTPS > 0) {
                PIDFCoefficients pidf = new PIDFCoefficients(TUNED_P, TUNED_I, TUNED_D, TUNED_F);
                launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                launchMotor.setVelocity(shooterTPS);
            } else {
                launchMotor.setVelocity(0.0);
                launchMotor.setPower(0.0);
            }
        }
    }

    private void postData(Telemetry dashTelemetry) {
        double actualTPS = (launchMotor != null) ? launchMotor.getVelocity() : 0.0;
        double error = shooterTPS - actualTPS;
        boolean isSettled = Math.abs(error) <= TPS_TOL;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Shooter TPS", shooterTPS);
        packet.put("Actual Shooter TPS", actualTPS);
        packet.put("Shooter Error", error);
        packet.put("Motor Power", launchMotor != null ? launchMotor.getPower() * 1000.0 : 0.0);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        dashTelemetry.addLine("PIDF Manual TeleOp Running");
        dashTelemetry.addData("Target Shooter TPS", "%.0f tps (dpad up/down +/- 25)", shooterTPS);
        dashTelemetry.addData("Current Shooter TPS", "%.0f tps", actualTPS);
        dashTelemetry.addData("Shooter Error", "%.0f tps", error);
        dashTelemetry.addData("Shooter Power", "%.3f", launchMotor != null ? launchMotor.getPower() : 0.0);
        dashTelemetry.addData("Shooter Settled", isSettled);
        dashTelemetry.addData("Is Shooting", shooting);
        dashTelemetry.addData("Gate Position", "%.2f", gatePos);
        dashTelemetry.addData("Tuned P", "%.5f", TUNED_P);
        dashTelemetry.addData("Tuned I", "%.5f", TUNED_I);
        dashTelemetry.addData("Tuned F", "%.5f", TUNED_F);
        dashTelemetry.update();
    }
}
