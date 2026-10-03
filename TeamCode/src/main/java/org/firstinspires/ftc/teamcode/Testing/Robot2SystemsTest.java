package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "SystemsTest", group = "Testing")
public class Robot2SystemsTest extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;
    private CRServo feedServo;
    private CRServo sideServo;

    // Controls:
    // A = Intake ON/OFF
    // B = Transfer ON/OFF
    // X = Feed/Cam-Jet ON/OFF
    // Y = Everything On
    // LB = Everything Off
    // RB = +5 TPS
    // RT = +10 TPS
    // LT = -10 TPS
    // D-Pad Up = +50 TPS
    // D-Pad Down = -50 TPS
    // D-Pad Left = Reset Flywheel to 1175 TPS

    // Flywheel PIDF from the previous robot
    private static final double FLYWHEEL_P = 500;
    private static final double FLYWHEEL_I = 3;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 4;

    // Flywheel speed settings
    private static final double STARTING_FLYWHEEL_TPS = 1175.0;

    private static final double INTAKE_POWER = 0.90;
    private static final double SIDE_POWER = 1.0;
    private static final double FEED_POWER = -0.9;

    private boolean intakeOn = false;
    private boolean sideOn = false;
    private boolean feedOn = false;

    private double targetFlywheelVelocity = STARTING_FLYWHEEL_TPS;

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;

    private double previousRightTrigger = 0;
    private double previousLeftTrigger = 0;

    private DcMotorEx getMotorWithFallback(String... names) {
        for (String name : names) {
            try {
                DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
                if (m != null) return m;
            } catch (Exception ignored) {}
        }
        return null;
    }

    @Override
    public void runOpMode() {
        leftFront = getMotorWithFallback("leftFront", "lf", "left_front");
        leftBack = getMotorWithFallback("leftBack", "leftRear", "lr", "left_back");
        rightFront = getMotorWithFallback("rightFront", "rf", "right_front");
        rightBack = getMotorWithFallback("rightBack", "rightRear", "rr", "right_back");

        if (leftFront != null) {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (leftBack != null) {
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (rightFront != null) {
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (rightBack != null) {
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        } catch (Exception ignored) {}

        try {
            launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        } catch (Exception ignored) {}

        try {
            feedServo = hardwareMap.get(CRServo.class, "feedServo");
        } catch (Exception ignored) {}

        try {
            sideServo = hardwareMap.get(CRServo.class, "sideServo");
        } catch (Exception ignored) {}

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(0);
        }

        if (launchMotor != null) {
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F));
            launchMotor.setPower(0);
        }

        if (feedServo != null) feedServo.setPower(0);
        if (sideServo != null) sideServo.setPower(0);

        telemetry.addLine("Systems Test Ready");
        telemetry.addData("Drive LF", leftFront != null ? "OK" : "MISSING");
        telemetry.addData("Drive LB", leftBack != null ? "OK" : "MISSING");
        telemetry.addData("Drive RF", rightFront != null ? "OK" : "MISSING");
        telemetry.addData("Drive RB", rightBack != null ? "OK" : "MISSING");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {

                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
                double lfPower = (drive + strafe + turn) / denominator;
                double lbPower = (drive - strafe + turn) / denominator;
                double rfPower = (drive - strafe - turn) / denominator;
                double rbPower = (drive + strafe - turn) / denominator;

                if (leftFront != null) leftFront.setPower(lfPower);
                if (leftBack != null) leftBack.setPower(lbPower);
                if (rightFront != null) rightFront.setPower(rfPower);
                if (rightBack != null) rightBack.setPower(rbPower);

                if (gamepad1.a && !previousA) {
                    intakeOn = !intakeOn;
                    if (intakeMotor != null) intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0);
                }

                if (gamepad1.b && !previousB) {
                    sideOn = !sideOn;
                    if (sideServo != null) sideServo.setPower(sideOn ? SIDE_POWER : 0);
                }

                if (gamepad1.x && !previousX) {
                    feedOn = !feedOn;
                    if (feedServo != null) feedServo.setPower(feedOn ? FEED_POWER : 0);
                }

                if (gamepad1.y && !previousY) {
                    intakeOn = true;
                    sideOn = true;
                    feedOn = true;

                    if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);
                    if (sideServo != null) sideServo.setPower(SIDE_POWER);
                    if (feedServo != null) feedServo.setPower(FEED_POWER);
                }

                if (gamepad1.left_bumper && !previousLeftBumper) {
                    intakeOn = false;
                    sideOn = false;
                    feedOn = false;

                    if (intakeMotor != null) intakeMotor.setPower(0);
                    if (sideServo != null) sideServo.setPower(0);
                    if (feedServo != null) feedServo.setPower(0);

                    targetFlywheelVelocity = 0;

                    if (launchMotor != null) launchMotor.setVelocity(0);
                }

                if (gamepad1.right_bumper && !previousRightBumper) {
                    targetFlywheelVelocity += 5;
                }

                if (gamepad1.right_trigger > 0.5 && previousRightTrigger <= 0.5) {
                    targetFlywheelVelocity += 10;
                }

                if (gamepad1.left_trigger > 0.5 && previousLeftTrigger <= 0.5) {
                    targetFlywheelVelocity -= 10;
                }

                if (gamepad1.dpad_up && !previousDpadUp) {
                    targetFlywheelVelocity += 50;
                }

                if (gamepad1.dpad_down && !previousDpadDown) {
                    targetFlywheelVelocity -= 50;
                }

                if (gamepad1.dpad_left && !previousDpadLeft) {
                    targetFlywheelVelocity = STARTING_FLYWHEEL_TPS;
                }

                targetFlywheelVelocity = Math.max(0, targetFlywheelVelocity);

                if (launchMotor != null) launchMotor.setVelocity(targetFlywheelVelocity);

                previousA = gamepad1.a;
                previousB = gamepad1.b;
                previousX = gamepad1.x;
                previousY = gamepad1.y;
                previousLeftBumper = gamepad1.left_bumper;
                previousRightBumper = gamepad1.right_bumper;
                previousRightTrigger = gamepad1.right_trigger;
                previousLeftTrigger = gamepad1.left_trigger;
                previousDpadUp = gamepad1.dpad_up;
                previousDpadDown = gamepad1.dpad_down;
                previousDpadLeft = gamepad1.dpad_left;

                telemetry.addLine("---- Systems Test ----");
                telemetry.addData("GP1 Joysticks", "LY:%.2f LX:%.2f RX:%.2f", drive, strafe, turn);
                telemetry.addData("Drive Init", "LF:%s LB:%s RF:%s RB:%s",
                        leftFront != null ? "OK" : "MISSING",
                        leftBack != null ? "OK" : "MISSING",
                        rightFront != null ? "OK" : "MISSING",
                        rightBack != null ? "OK" : "MISSING");
                telemetry.addData("Drive Powers", "LF:%.2f LB:%.2f RF:%.2f RB:%.2f",
                        lfPower, lbPower, rfPower, rbPower);
                telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
                telemetry.addData("Transfer", sideOn ? "ON" : "OFF");
                telemetry.addData("Feed", feedOn ? "ON" : "OFF");
                telemetry.addData("Flywheel Target TPS", "%.0f", targetFlywheelVelocity);
                telemetry.addData("Flywheel Actual TPS", "%.0f",
                        launchMotor != null ? launchMotor.getVelocity() : 0);
                telemetry.update();
            }
        } finally {
            if (leftFront != null) leftFront.setPower(0);
            if (leftBack != null) leftBack.setPower(0);
            if (rightFront != null) rightFront.setPower(0);
            if (rightBack != null) rightBack.setPower(0);
            if (intakeMotor != null) intakeMotor.setPower(0);

            if (launchMotor != null) {
                launchMotor.setVelocity(0);
                launchMotor.setPower(0);
            }

            if (feedServo != null) feedServo.setPower(0);
            if (sideServo != null) sideServo.setPower(0);
        }
    }
}