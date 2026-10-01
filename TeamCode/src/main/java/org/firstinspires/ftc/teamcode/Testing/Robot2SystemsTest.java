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
    // D-Pad Up = 720 TPS
    // D-Pad Right = 1175 TPS
    // D-Pad Down = 1550 TPS
    // D-Pad Left = Flywheel OFF

    // Flywheel PIDF from the previous robot
    private static final double FLYWHEEL_P = 500;
    private static final double FLYWHEEL_I = 3;
    private static final double FLYWHEEL_D = 0;
    private static final double FLYWHEEL_F = 4;

    // Previous flywheel speeds
    private static final double FLYWHEEL_720 = 720.0;
    private static final double FLYWHEEL_SHORT = 1175.0;
    private static final double FLYWHEEL_LONG = 1550.0;

    private static final double INTAKE_POWER = 0.90;
    private static final double SIDE_POWER = 1.0;
    private static final double FEED_POWER = -0.9;

    private boolean intakeOn = false;
    private boolean sideOn = false;
    private boolean feedOn = false;

    private double targetFlywheelVelocity = 0;

    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousLeftBumper = false;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        feedServo = hardwareMap.get(CRServo.class, "feedServo");
        sideServo = hardwareMap.get(CRServo.class, "sideServo");

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F));

        intakeMotor.setPower(0);
        launchMotor.setPower(0);
        feedServo.setPower(0);
        sideServo.setPower(0);

        telemetry.addLine("Systems Test Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {
            while (opModeIsActive()) {

                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
                leftFront.setPower((drive + strafe + turn) / denominator);
                leftBack.setPower((drive - strafe + turn) / denominator);
                rightFront.setPower((drive - strafe - turn) / denominator);
                rightBack.setPower((drive + strafe - turn) / denominator);

                if (gamepad1.a && !previousA) {
                    intakeOn = !intakeOn;
                    intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0);
                }

                if (gamepad1.b && !previousB) {
                    sideOn = !sideOn;
                    sideServo.setPower(sideOn ? SIDE_POWER : 0);
                }

                if (gamepad1.x && !previousX) {
                    feedOn = !feedOn;
                    feedServo.setPower(feedOn ? FEED_POWER : 0);
                }

                if (gamepad1.y && !previousY) {
                    intakeOn = true;
                    sideOn = true;
                    feedOn = true;

                    intakeMotor.setPower(INTAKE_POWER);
                    sideServo.setPower(SIDE_POWER);
                    feedServo.setPower(FEED_POWER);
                }

                if (gamepad1.left_bumper && !previousLeftBumper) {
                    intakeOn = false;
                    sideOn = false;
                    feedOn = false;

                    intakeMotor.setPower(0);
                    sideServo.setPower(0);
                    feedServo.setPower(0);

                    targetFlywheelVelocity = 0;
                    launchMotor.setVelocity(0);
                }

                if (gamepad1.dpad_up) targetFlywheelVelocity = FLYWHEEL_720;
                if (gamepad1.dpad_right) targetFlywheelVelocity = FLYWHEEL_SHORT;
                if (gamepad1.dpad_down) targetFlywheelVelocity = FLYWHEEL_LONG;
                if (gamepad1.dpad_left) targetFlywheelVelocity = 0;

                launchMotor.setVelocity(targetFlywheelVelocity);

                previousA = gamepad1.a;
                previousB = gamepad1.b;
                previousX = gamepad1.x;
                previousY = gamepad1.y;
                previousLeftBumper = gamepad1.left_bumper;

                telemetry.addLine("Systems Test");
                telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
                telemetry.addData("Transfer", sideOn ? "ON" : "OFF");
                telemetry.addData("Feed", feedOn ? "ON" : "OFF");
                telemetry.addData("Flywheel Target TPS", "%.0f", targetFlywheelVelocity);
                telemetry.addData("Flywheel Actual TPS", "%.0f", launchMotor.getVelocity());
                telemetry.update();
            }
        } finally {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            intakeMotor.setPower(0);
            launchMotor.setVelocity(0);
            launchMotor.setPower(0);
            feedServo.setPower(0);
            sideServo.setPower(0);
        }
    }
}