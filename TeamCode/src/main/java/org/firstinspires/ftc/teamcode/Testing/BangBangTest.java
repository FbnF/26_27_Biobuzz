package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controls (Gamepad 1):
// A Button - Start/Stop Flywheel
// B Button - Stop Flywheel
// X Button - Toggle Intake ON/OFF
// Y Button - Toggle Transfer (Side Servo) ON/OFF
// Right Bumper (RB) - Toggle Cam-Jet (Feed Servo) ON/OFF
// Left Bumper (LB) - Everything OFF
// D-Pad Up / Down - Adjust Target Speed (+/- 50 TPS)
@Config
@TeleOp(name = "FlywheelBangBangTest", group = "Testing")
public class BangBangTest extends LinearOpMode {

    // Parameters (Tunable via FTC Dashboard / Panels)
    public static double TARGET_TPS = 1165.0;
    public static double HIGH_POWER = 1.0;
    public static double LOW_POWER = 0.0;
    public static double COAST_POWER = 0.4;
    public static double HYSTERESIS_TPS = 15.0;
    public static boolean USE_COAST_POWER = false;
    public static double TPS_TOL = 50.0;
    public static boolean ENABLE_CSV_LOGGING = true;

    public static double INTAKE_POWER = 0.90;
    public static double SIDE_POWER = 1.0;
    public static double FEED_POWER = -0.9;

    private boolean isRunning = false;
    private boolean intakeOn = false;
    private boolean sideOn = false;
    private boolean feedOn = false;

    private long motorStartNs = 0;
    private double timeToTargetSec = -1.0;

    private DcMotorEx launchMotor;
    private DcMotorEx intakeMotor;
    private CRServo sideServo;
    private CRServo feedServo;

    private FlywheelTestLogger logger;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void runOpMode() {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        try { sideServo = hardwareMap.get(CRServo.class, "sideServo"); } catch (Exception ignored) {}
        try { feedServo = hardwareMap.get(CRServo.class, "feedServo"); } catch (Exception ignored) {}

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (launchMotor != null) {
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            launchMotor.setPower(0);
        }

        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(0);
        }

        if (sideServo != null) sideServo.setPower(0);
        if (feedServo != null) feedServo.setPower(0);

        dashTelemetry.addLine("Bang-Bang Test Ready");
        dashTelemetry.addData("Launch Motor Init", launchMotor != null ? "ALL GOOD" : "MISSING");
        dashTelemetry.addData("Intake Motor Init", intakeMotor != null ? "ALL GOOD" : "MISSING");
        dashTelemetry.addData("Side Servo Init", sideServo != null ? "ALL GOOD" : "MISSING");
        dashTelemetry.addData("Feed Servo Init", feedServo != null ? "ALL GOOD" : "MISSING");
        dashTelemetry.addLine("Controls: A-Flywheel Start, B-Flywheel Stop, X-Intake, Y-Transfer, RB-Feed, LB-All OFF");
        dashTelemetry.update();

        waitForStart();

        if (ENABLE_CSV_LOGGING) {
            logger = new FlywheelTestLogger("BANGBANGONLY");
        }

        try {
            while (opModeIsActive()) {

                // Flywheel Start / Stop
                if (gamepad1.a && !prevA) {
                    isRunning = true;
                    motorStartNs = System.nanoTime();
                    timeToTargetSec = -1.0;
                }

                if (gamepad1.b && !prevB) {
                    isRunning = false;
                    timeToTargetSec = -1.0;
                }

                // Intake Toggle (X)
                if (gamepad1.x && !prevX) {
                    intakeOn = !intakeOn;
                    if (intakeMotor != null) intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0);
                }

                // Transfer / Side Servo Toggle (Y)
                if (gamepad1.y && !prevY) {
                    sideOn = !sideOn;
                    if (sideServo != null) sideServo.setPower(sideOn ? SIDE_POWER : 0);
                }

                // Cam-Jet / Feed Servo Toggle (Right Bumper)
                if (gamepad1.right_bumper && !prevRB) {
                    feedOn = !feedOn;
                    if (feedServo != null) feedServo.setPower(feedOn ? FEED_POWER : 0);
                }

                // Emergency Everything OFF (Left Bumper)
                if (gamepad1.left_bumper && !prevLB) {
                    isRunning = false;
                    intakeOn = false;
                    sideOn = false;
                    feedOn = false;

                    if (intakeMotor != null) intakeMotor.setPower(0);
                    if (sideServo != null) sideServo.setPower(0);
                    if (feedServo != null) feedServo.setPower(0);
                    if (launchMotor != null) launchMotor.setPower(0);
                }

                // Target Speed Adjustment (D-Pad)
                if (gamepad1.dpad_up && !prevUp) {
                    TARGET_TPS += 50.0;
                }

                if (gamepad1.dpad_down && !prevDown) {
                    TARGET_TPS = Math.max(0.0, TARGET_TPS - 50.0);
                }

                prevA = gamepad1.a;
                prevB = gamepad1.b;
                prevX = gamepad1.x;
                prevY = gamepad1.y;
                prevRB = gamepad1.right_bumper;
                prevLB = gamepad1.left_bumper;
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

                // Send live graph packet to FTC Dashboard
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Target TPS", TARGET_TPS);
                packet.put("Actual TPS", actualTPS);
                packet.put("Velocity Error", velocityError);
                packet.put("Motor Power", motorPower * 1000.0);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // Telemetry text output
                dashTelemetry.addLine("---- Bang-Bang Only Flywheel Test ----");
                dashTelemetry.addData("Controller Mode", "BANGBANGONLY");
                dashTelemetry.addData("Flywheel Status", isRunning ? "ITS RUNNING" : "ITS STOPPED :(");
                dashTelemetry.addData("Intake Status", intakeOn ? "ON (0.90)" : "OFF");
                dashTelemetry.addData("Transfer Status", sideOn ? "ON (1.00)" : "OFF");
                dashTelemetry.addData("Feed Status", feedOn ? "ON (-0.90)" : "OFF");
                dashTelemetry.addData("Target TPS", "%.1f", TARGET_TPS);
                dashTelemetry.addData("Actual TPS", "%.1f", actualTPS);
                dashTelemetry.addData("Velocity Error", "%.1f TPS", velocityError);
                dashTelemetry.addData("Motor Output Power", "%.3f", motorPower);
                dashTelemetry.addData("Time Elapsed", "%.2f s", elapsedSec);
                dashTelemetry.addData("Time To Target", timeToTargetSec >= 0 ? "%.2f s" : "NOT_REACHED", timeToTargetSec);
                dashTelemetry.addData("Settled Status", isSettled ? "SETTLED (READY)" : "SPOOLING");
                dashTelemetry.addData("USB Log File", logger != null ? logger.getRelativePath() : "DISABLED");
                dashTelemetry.update();
            }
        } finally {
            if (launchMotor != null) launchMotor.setPower(0);
            if (intakeMotor != null) intakeMotor.setPower(0);
            if (sideServo != null) sideServo.setPower(0);
            if (feedServo != null) feedServo.setPower(0);
            if (logger != null) logger.close();
        }
    }
}
