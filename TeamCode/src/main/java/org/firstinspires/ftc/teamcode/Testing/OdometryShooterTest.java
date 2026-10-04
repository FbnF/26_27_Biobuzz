package org.firstinspires.ftc.teamcode.Testing;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.TeleOp.OdometryShooterService;
import org.firstinspires.ftc.teamcode.TeleOp.ShootingState;

// Test OpMode for dynamic flywheel calculations and pose simulation
// Controls:
// Left Stick Y  : Move Simulated Robot Y
// Left Stick X  : Move Simulated Robot X
// Right Stick X : Rotate Simulated Robot Heading
// A Button      : Toggle GoingToShoot (ON/OFF)
// B Button      : Toggle Alliance Target (BLUE/RED)
// X Button      : Reset Pose to OpModeStorage
@TeleOp(name = "OdometryShooterTest", group = "Testing")
public class OdometryShooterTest extends LinearOpMode {

    private OdometryShooterService shooterService;

    // Simulated robot pose
    private double simX = 72.0;
    private double simY = 12.0;
    private double simHeadingRad = Math.toRadians(90.0);

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {
        shooterService = OdometryShooterService.getInstance();

        // Load pose from auto storage if available
        if (OpModeStorage.autonomousEndPose != null) {
            simX = OpModeStorage.autonomousEndPose.x();
            simY = OpModeStorage.autonomousEndPose.y();
            simHeadingRad = OpModeStorage.autonomousEndPose.heading();
        }

        telemetry.addLine("Odometry Shooter Test Ready.");
        telemetry.addLine("A: Toggle Calc | B: Toggle Alliance | X: Reset Pose");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Joystick pose movement speed
            double moveSpeed = 0.5;
            double turnSpeed = Math.toRadians(2.0);

            simY += -gamepad1.left_stick_y * moveSpeed;
            simX += gamepad1.left_stick_x * moveSpeed;
            simHeadingRad += gamepad1.right_stick_x * turnSpeed;

            // Keep heading in [-PI, PI]
            simHeadingRad = Math.atan2(Math.sin(simHeadingRad), Math.cos(simHeadingRad));

            Pose currentSimPose = new Pose(simX, simY, simHeadingRad);

            // Controls
            if (gamepad1.a && !prevA) {
                shooterService.toggleGoingToShoot();
            }
            if (gamepad1.b && !prevB) {
                shooterService.setBlueAlliance(!shooterService.isBlueAlliance());
            }
            if (gamepad1.x && !prevX && OpModeStorage.autonomousEndPose != null) {
                simX = OpModeStorage.autonomousEndPose.x();
                simY = OpModeStorage.autonomousEndPose.y();
                simHeadingRad = OpModeStorage.autonomousEndPose.heading();
            }

            prevA = gamepad1.a;
            prevB = gamepad1.b;
            prevX = gamepad1.x;

            // Evaluate shooting calculations
            ShootingState state = shooterService.evaluateShooting(currentSimPose);

            // Telemetry Output
            telemetry.addLine("---- Pose Simulation ----");
            telemetry.addData("Sim Pose (X, Y)", "%.1f in, %.1f in", currentSimPose.x(), currentSimPose.y());
            telemetry.addData("Sim Heading", "%.1f deg", Math.toDegrees(currentSimPose.heading()));

            telemetry.addLine("---- Service Mode ----");
            telemetry.addData("Alliance Target", shooterService.isBlueAlliance() ? "BLUE HIVE" : "RED HIVE");
            telemetry.addData("GoingToShoot Toggle", shooterService.isGoingToShoot() ? "ACTIVE (ON)" : "DISABLED (OFF)");

            telemetry.addLine("---- Calculation Results ----");
            telemetry.addData("Valid Sector Zone", state.isValidZone ? "YES (READY)" : "NO (" + state.statusMessage + ")");
            telemetry.addData("Distance to Hive", "%.1f in", state.distanceInches);
            telemetry.addData("Target Heading", "%.1f deg", Math.toDegrees(state.targetHeadingRad));
            telemetry.addData("Heading Error", "%.1f deg", Math.toDegrees(state.headingErrorRad));
            telemetry.addData("Target Flywheel TPS", "%.0f TPS", state.targetTPS);

            telemetry.update();
        }
    }
}
