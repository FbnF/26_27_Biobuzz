package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.LimelightShooterConfig;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.Services.TurretService;
import org.firstinspires.ftc.teamcode.Services.VisionService;

@Config
@TeleOp(name = "TurretLimelightShooting", group = "Testing")
public class TurretLimelightShooting extends LinearOpMode {

    public static double MANUAL_TPS_OFFSET = 0.0;
    public static double MANUAL_ANGLE_OFFSET = 0.0;
    public static boolean AUTO_TRACKING_ENABLED = true;

    private VisionService visionService;
    private TurretService turretService;

    private DcMotorEx launchMotor;
    private DcMotorEx turretMotor;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevUp = false;
    private boolean prevDown = false;

    @Override
    public void runOpMode() {
        Telemetry dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        visionService = new VisionService();
        turretService = new TurretService();

        try {
            launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        } catch (Exception ignored) {
            launchMotor = null;
        }

        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        } catch (Exception ignored) {
            turretMotor = null;
        }

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (launchMotor != null) {
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            launchMotor.setPower(0);
        }

        dashTelemetry.addLine("TurretLimelightShooting Ready");
        dashTelemetry.addData("Launch Motor", launchMotor != null ? "OK" : "MISSING");
        dashTelemetry.addData("Turret Motor", turretMotor != null ? "OK" : "MISSING");
        dashTelemetry.update();

        visionService.start(hardwareMap);

        if (turretMotor != null) {
            turretService.initTurret(turretMotor);
        }

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Read vision data
                VisionService.Reading reading = visionService.getLatest();

                // Gamepad 1 Controls
                if (gamepad1.a && !prevA) {
                    AUTO_TRACKING_ENABLED = !AUTO_TRACKING_ENABLED;
                }

                if (gamepad1.b && !prevB) {
                    AUTO_TRACKING_ENABLED = false;
                    if (launchMotor != null) launchMotor.setPower(0);
                    turretService.stopTurret();
                }

                if (gamepad1.x && !prevX) {
                    turretService.setTargetAngle(TurretConfig.min);
                }

                if (gamepad1.dpad_up && !prevUp) {
                    MANUAL_TPS_OFFSET += 25.0;
                }

                if (gamepad1.dpad_down && !prevDown) {
                    MANUAL_TPS_OFFSET -= 25.0;
                }

                prevA = gamepad1.a;
                prevB = gamepad1.b;
                prevX = gamepad1.x;
                prevUp = gamepad1.dpad_up;
                prevDown = gamepad1.dpad_down;

                double targetTPS = 0.0;
                double targetAngle = TurretConfig.min;
                double actualTPS = (launchMotor != null) ? launchMotor.getVelocity() : 0.0;

                if (AUTO_TRACKING_ENABLED && reading.hasTag && reading.smoothedDistanceIn >= LimelightShooterConfig.MIN_RANGE_IN) {
                    double dist = reading.smoothedDistanceIn;

                    double[] distTable;
                    double[] tpsTable;
                    double[] angleTable;

                    if (LimelightShooterConfig.isGroupB(reading.id)) {
                        distTable = LimelightShooterConfig.DIST_IN_GROUP_B;
                        tpsTable = LimelightShooterConfig.TPS_AT_DIST_GROUP_B;
                        angleTable = LimelightShooterConfig.TURN_ANGLE_GROUP_B;
                    } else {
                        distTable = LimelightShooterConfig.DIST_IN_GROUP_A;
                        tpsTable = LimelightShooterConfig.TPS_AT_DIST_GROUP_A;
                        angleTable = LimelightShooterConfig.TURN_ANGLE_GROUP_A;
                    }

                    targetTPS = LimelightShooterConfig.interpolate(dist, distTable, tpsTable) + MANUAL_TPS_OFFSET;
                    targetAngle = LimelightShooterConfig.interpolate(dist, distTable, angleTable) + MANUAL_ANGLE_OFFSET;

                    targetAngle = Math.max(TurretConfig.min, Math.min(TurretConfig.max, targetAngle));

                    if (launchMotor != null) {
                        launchMotor.setVelocity(targetTPS);
                    }

                    turretService.setTargetAngle(targetAngle);
                    turretService.updateTurret();
                } else if (!AUTO_TRACKING_ENABLED) {
                    if (launchMotor != null) launchMotor.setPower(0);
                    turretService.stopTurret();
                } else {
                    // Tag lost or out of range
                    turretService.updateTurret();
                }

                // Send live FTC Dashboard graph packet
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Vision Has Tag", reading.hasTag ? 1 : 0);
                packet.put("Tag Distance (In)", reading.smoothedDistanceIn);
                packet.put("Target TPS", targetTPS);
                packet.put("Actual TPS", actualTPS);
                packet.put("Target Angle (Rad)", targetAngle);
                packet.put("Turret Angle Error (Rad)", (targetAngle - (turretService.getCurrentTicks() / TurretConfig.TICKS_PER_RADIAN)));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // Driver Station Telemetry Output
                dashTelemetry.addLine("---- Turret Limelight Shooting ----");
                dashTelemetry.addData("Tracking Status", AUTO_TRACKING_ENABLED ? "AUTO TRACKING ON" : "PAUSED");
                dashTelemetry.addData("Vision Tag Visible", reading.hasTag ? "TAG #" + reading.id : "NO TAG");
                dashTelemetry.addData("Smoothed Distance", "%.2f in", reading.smoothedDistanceIn);
                dashTelemetry.addData("Target TPS", "%.1f (Offset: %.1f)", targetTPS, MANUAL_TPS_OFFSET);
                dashTelemetry.addData("Actual TPS", "%.1f", actualTPS);
                dashTelemetry.addData("Target Angle", "%.3f rad (Offset: %.3f)", targetAngle, MANUAL_ANGLE_OFFSET);
                dashTelemetry.addData("Turret Current Ticks", turretService.getCurrentTicks());
                dashTelemetry.addData("Turret Target Ticks", turretService.getTargetTicks());
                dashTelemetry.addData("Turret At Target", turretService.atTarget());
                dashTelemetry.update();
            }
        } finally {
            if (launchMotor != null) launchMotor.setPower(0);
            turretService.stopTurret();
            visionService.stop();
        }
    }
}
