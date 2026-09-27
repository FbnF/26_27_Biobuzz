package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "FlywheelTest", group = "Testing")
public class FlywheelTest extends LinearOpMode {

    private DcMotorEx motor;
    double targetTPS;

    @Override
    public void runOpMode() {
        // "motor" must match the name you gave this motor in the
        // Robot Controller's Configure Robot Hardware screen.
        motor = hardwareMap.get(DcMotorEx.class, "motor");


        waitForStart();

        while (opModeIsActive()) {
            // Left stick up = negative value, so we flip the sign
            // to make "up" drive the motor forward.
            if (gamepad1.a) targetTPS = 1300;
            if (gamepad1.b) targetTPS = 1325;
            if (gamepad1.x) targetTPS = 1350;
            if (gamepad1.y) targetTPS = 1400;
            if (gamepad1.right_bumper) targetTPS=1375;
            if (gamepad1.left_bumper) targetTPS=1325;



            motor.setVelocity(targetTPS);


        }

    }
}
