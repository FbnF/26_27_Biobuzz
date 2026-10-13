package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.TurretConfig;
import org.firstinspires.ftc.teamcode.Services.TurretService;

// Controls:
// LT / RT      = nudge target angle down / up by STEP
// D-Pad Up/Down = increase / decrease STEP
// A / Y        = target min / max angle
// X            = stop turret
// Tune POWER, DEADBAND, HYSTERESIS, TICKS_PER_RADIAN live via FTC Dashboard.
@TeleOp(name = "TurretBangBangTest", group = "Testing")
public class TurretBangBangTest extends LinearOpMode {

    private static final double ANGLE_STEP = 0.05;

    private TurretService turretService;
    private DcMotorEx turretMotor;

    private double targetAngle = 0.0;
    private double step = ANGLE_STEP;

    @Override
    public void runOpMode() {
        turretService = new TurretService();

        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        } catch (Exception ignored) {
            turretMotor = null;
        }

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("turretMotor", turretMotor != null ? "OK" : "MISSING");
        telemetry.update();

        waitForStart();

        if (isStopRequested() || turretMotor == null) return;

        turretService.initTurret(turretMotor);

        try {
            while (opModeIsActive()) {
                if (gamepad1.leftTriggerWasPressed()) {
                    targetAngle -= step;
                }
                if (gamepad1.rightTriggerWasPressed()) {
                    targetAngle += step;
                }
                if (gamepad1.dpad_up) {
                    step += 0.01;
                }
                if (gamepad1.dpad_down) {
                    step = Math.max(0.01, step - 0.01);
                }
                if (gamepad1.aWasPressed()) {
                    targetAngle = TurretConfig.min;
                }
                if (gamepad1.yWasPressed()) {
                    targetAngle = TurretConfig.max;
                }
                if (gamepad1.xWasPressed()) {
                    turretService.stopTurret();
                }

                targetAngle = Math.max(TurretConfig.min, Math.min(TurretConfig.max, targetAngle));
                turretService.setTargetAngle(targetAngle);
                turretService.updateTurret();

                telemetry.addLine("---- Turret Bang-Bang ----");
                telemetry.addData("Target Angle (rad)", "%.3f", targetAngle);
                telemetry.addData("Step (rad)", "%.3f", step);
                telemetry.addData("Target Ticks", turretService.getTargetTicks());
                telemetry.addData("Current Ticks", turretService.getCurrentTicks());
                telemetry.addData("Error", turretService.getTargetTicks() - turretService.getCurrentTicks());
                telemetry.addData("At Target", turretService.atTarget());
                telemetry.addLine();
                telemetry.addData("POWER", "%.2f", TurretConfig.BANG_BANG_POWER);
                telemetry.addData("DEADBAND", "%.1f", TurretConfig.BANG_BANG_DEADBAND);
                telemetry.addData("HYSTERESIS", "%.1f", TurretConfig.BANG_BANG_HYSTERESIS);
                telemetry.addData("TICKS_PER_RADIAN", "%.1f", TurretConfig.TICKS_PER_RADIAN);
                telemetry.update();
            }
        } finally {
            turretService.stopTurret();
        }
    }
}
