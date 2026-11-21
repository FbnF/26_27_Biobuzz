package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BangBangManualTeleop", group = "teleop")
public class BangBangManualTeleop extends LinearOpMode {

    private Follower follower;
    private DcMotorEx launchMotor;
    private DcMotorEx turretMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx intakeMotor;
    private Servo gateServo;
    private CRServo contServo;

    // Bang-Bang Control Parameters
    public static double HIGH_POWER = 1.0;
    public static double LOW_POWER = 0.0;
    public static double HYSTERESIS_TPS = 15.0;
    public static double TPS_TOL = 50.0;

    private boolean shooting = false;
    private double gatePos = 0.5;
    private double intakeTPS = 0;
    private double shooterTPS = 1200;
    private double curTransferTps;
    private double turretPower;
    private double baseTransferTps = 1150;

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

        follower = Constants.create(hardwareMap);

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        contServo = hardwareMap.get(CRServo.class, "contServo");

        contServo.setDirection(CRServo.Direction.REVERSE);

        curTransferTps = baseTransferTps;
        contServo.setPower(0);
        gatePos = 0.11;

        if (launchMotor != null) {
            launchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            launchMotor.setPower(0);
        }

        waitForStart();

        if (isStopRequested()) return;

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
            postData();

            updateButtonStates();
        }

        if (launchMotor != null) launchMotor.setPower(0);
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
            transferMotor.setPower(1.0);
            intakeTPS = 0.9;
        }
        if (leftTriggerPressed && !shooting) {
            transferMotor.setPower(0.8);
            intakeTPS = 0.0;
        }
        if (leftBumperPressed && !shooting) {
            transferMotor.setPower(-1.0);
            intakeTPS = -0.9;
        }
    }

    private void tpsAdjustments() {
        boolean dpadRightPressed = gamepad2.dpad_right && !lastDpadRight2;
        boolean dpadLeftPressed = gamepad2.dpad_left && !lastDpadLeft2;
        boolean dpadUpPressed = gamepad2.dpad_up && !lastDpadUp2;
        boolean dpadDownPressed = gamepad2.dpad_down && !lastDpadDown2;

        if (dpadRightPressed && !shooting) curTransferTps += 15;
        if (dpadLeftPressed && !shooting) curTransferTps -= 15;
        if (dpadUpPressed) shooterTPS += 50;
        if (dpadDownPressed) shooterTPS -= 50;
    }

    private void shooterLogic() {
        boolean yPressed = gamepad2.y && !lastY2;
        boolean xPressed = gamepad2.x && !lastX2;

        if (yPressed) {
            shooting = true;
            transferMotor.setPower(1.0);
            intakeTPS = 0.9;
            gatePos = 0.35;
            contServo.setPower(1.0);
        }
        if (xPressed) {
            shooting = false;
            transferMotor.setPower(1.0);
            intakeTPS = 0.9;
            gatePos = 0.11;
            contServo.setPower(0);
        }
        turretPower = gamepad2.right_stick_x;
    }

    private void settingPowers() {
        gateServo.setPosition(gatePos);
        intakeMotor.setPower(intakeTPS);
        turretMotor.setPower(turretPower);

        if (launchMotor != null) {
            if (shooterTPS > 0) {
                double actualTPS = launchMotor.getVelocity();
                double motorPower;

                if (actualTPS < (shooterTPS - HYSTERESIS_TPS)) {
                    motorPower = HIGH_POWER;
                } else if (actualTPS >= (shooterTPS + HYSTERESIS_TPS)) {
                    motorPower = LOW_POWER;
                } else {
                    motorPower = launchMotor.getPower();
                }

                launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launchMotor.setPower(motorPower);
            } else {
                launchMotor.setPower(0);
            }
        }
    }

    private void postData() {
        double actualTPS = launchMotor != null ? launchMotor.getVelocity() : 0.0;
        double error = shooterTPS - actualTPS;
        boolean isSettled = Math.abs(error) <= TPS_TOL;

        telemetry.addData("Target ShooterTPS: ", shooterTPS);
        telemetry.addData("Current ShooterTPS: ", actualTPS);
        telemetry.addData("Shooter Power: ", launchMotor != null ? launchMotor.getPower() : 0.0);
        telemetry.addData("Shooter Settled: ", isSettled);
        telemetry.addData("Is Shooting: ", shooting);
        telemetry.addData("Gate Open: ", gatePos == 0.35);
        telemetry.update();
    }
}