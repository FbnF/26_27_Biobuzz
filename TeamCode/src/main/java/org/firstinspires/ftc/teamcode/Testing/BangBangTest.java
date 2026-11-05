package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//Controls
//A button - start motor
//B button - stop motor
// d pad up or down - adjust target speed by 50
@Config
@TeleOp(name = "FlywheelBangBangTest", group = "Testing")
public class BangBangTest extends LinearOpMode {

    //Paremeters
    public static double TARGET_TPS = 1165.0;
    public static double HIGH_POWER = 1.0;
    public static double LOW_POWER = 0.0;
    public static double COAST_POWER = 0.4;
    public static double HYSTERESIS_TPS = 15.0;
    public static boolean USE_COAST_POWER = false;
    public static double TPS_TOL = 50.0;
    public static boolean ENABLE_CSV_LOGGING = true;

    private boolean isRunning = false;
    private long motorStartNs = 0;
    private double timeToTargetSec = -1.0;
    private DcMotorEx launchMotor;
    private FlywheelTestLogger logger;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (launchMotor != null) {
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            launchMotor.setPower(0);
        }

        telemetry.addLine("Flywheel Test Ready");
        telemetry.addData("Motor Init", launchMotor != null ? "ALL GOOD" : "OH NOT ITS GONE");
        telemetry.addLine("Controls: A - Start, B - Stop, D pad up or down - speed up or down");
        telemetry.update();

        waitForStart();

        if (ENABLE_CSV_LOGGING) {
            logger = new FlywheelTestLogger("BANGBANGONLY");

        }

        try {
            while (opModeIsActive()) {
                if (gamepad1.a && !prevA) {
                    isRunning = true;
                    motorStartNs = System.nanoTime();
                    timeToTargetSec = -1.0;
                }

                if (gamepad1.b && !prevB) {
                    isRunning = false;
                    timeToTargetSec = -1.0;
                }

                if (gamepad1.dpad_up && !prevUp) {
                    TARGET_TPS += 50.0;
                }

                if (gamepad1.dpad_down && !prevDown) {
                    TARGET_TPS = Math.max(0.0, TARGET_TPS - 50.0);
                }

                prevA = gamepad1.a;
                prevB = gamepad1.b;
                prevUp = gamepad1.dpad_up;
                prevDown = gamepad1.dpad_down;

                double actualTPS = 0.0;
                double motorPower = 0.0;
                double elapsedSec = isRunning ? (System.nanoTime() - motorStartNs) / 1e9 : 0.0;

                if (isRunning && launchMotor != null) {
                    actualTPS = launchMotor.getVelocity();

                    double lowTargetPower = USE_COAST_POWER ? COAST_POWER : LOW_POWER;

                    if (actualTPS < (TARGET_TPS - HYSTERESIS_TPS)) {
                        motorPower = HIGH_POWER;
                    } else if (actualTPS >= (TARGET_TPS + HYSTERESIS_TPS)) {
                        motorPower = lowTargetPower;
                    } else {
                        motorPower = launchMotor.getPower();
                    }

                    launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    launchMotor.setPower(motorPower);

                    double error = TARGET_TPS - actualTPS;
                    boolean isSettled = Math.abs(error) <= TPS_TOL;

                    if (isSettled && timeToTargetSec < 0) {
                        timeToTargetSec = elapsedSec;
                    }

                    if (logger != null) {
                        logger.logRow(TARGET_TPS, actualTPS, motorPower, timeToTargetSec, isSettled);
                    }
                } else if (launchMotor != null) {
                    launchMotor.setPower(0);
                }

                double velocityError = isRunning ? (TARGET_TPS - actualTPS) : 0.0;
                boolean isSettled = isRunning && Math.abs(velocityError) <= TPS_TOL;

                telemetry.addLine("---- Bang-Bang Only Flywheel Test ----");
                telemetry.addData("Controller Mode", "BANGBANGONLY");
                telemetry.addData("Motor Status", isRunning ? "ITS RUNNING" : "ITS STOPPED :(");
                telemetry.addData("Target TPS", "%.1f", TARGET_TPS);
                telemetry.addData("Actual TPS", "%.1f", actualTPS);
                telemetry.addData("Velocity Error", "%.1f TPS", velocityError);
                telemetry.addData("Motor Output Power", "%.3f", motorPower);
                telemetry.addData("Time Elapsed", "%.2f s", elapsedSec);
                telemetry.addData("Time To Target", timeToTargetSec >= 0 ? "%.2f s" : "NOT_REACHED", timeToTargetSec);
                telemetry.addData("Settled Status", isSettled ? "SETTLED (READY)" : "SPOOLING");
                telemetry.update();
            }
        } finally {
            if (launchMotor != null) {
                launchMotor.setPower(0);
            }
            if (logger != null) {
                logger.close();
            }
        }

    }
}