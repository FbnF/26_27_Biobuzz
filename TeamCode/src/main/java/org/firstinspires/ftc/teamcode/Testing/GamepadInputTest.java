package org.firstinspires.ftc.teamcode.Testing;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.doclint.Messages;

@TeleOp(name = "Gamepad Input Test", group = "Testing")
public class GamepadInputTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Press play, then press/move every");
        telemetry.addLine("control on the gamepad to test it.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("===== GAMEPAD 1 =====");

            // --- Face buttons ---
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);

            // --- D-pad ---
            telemetry.addData("Dpad Up", gamepad1.dpad_up);
            telemetry.addData("Dpad Down", gamepad1.dpad_down);
            telemetry.addData("Dpad Left", gamepad1.dpad_left);
            telemetry.addData("Dpad Right", gamepad1.dpad_right);

            // --- Bumpers / Triggers ---
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);

            // --- Sticks ---
            telemetry.addData("Left Stick X", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", "%.2f", gamepad1.right_stick_y);
            telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
            telemetry.addData("Right Stick Button", gamepad1.right_stick_button);

            // --- Misc buttons ---
            telemetry.addData("Start", gamepad1.start);
            telemetry.addData("Back/Select", gamepad1.back);
            telemetry.addData("Guide", gamepad1.guide);

            telemetry.update();
        }
    }
}