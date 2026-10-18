package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "FlywheelTest", group = "Testing")
public class FlywheelTest extends LinearOpMode {

    private DcMotorEx motor;
    private Servo servo;
    double targetTPS;
    double position = 1;

    @Override
    public void runOpMode() {
        // "motor" must match the name you gave this motor in the
        // Robot Controller's Configure Robot Hardware screen.
        motor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        servo = hardwareMap.get(Servo.class, "turretServo");


        motor.setDirection(DcMotorEx.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            // Left stick up = negative value, so we flip the sign
            // to make "up" drive the motor forward.
            if (gamepad1.a) targetTPS = 1300;
            if (gamepad1.b) targetTPS = 1325;
            if (gamepad1.x) targetTPS = 1350;
            if (gamepad1.y) targetTPS = 1400;
            if (gamepad1.rightBumperWasPressed()) targetTPS-=50;
            if (gamepad1.leftBumperWasPressed()) targetTPS+=50;
            if(gamepad1.rightTriggerWasPressed()){
                position = 1;
                servo.setPosition(position);
            }
            if(gamepad1.leftTriggerWasPressed()){
                position = 0;
                servo.setPosition(position);
            }

            telemetry.addData("Turret Speed", targetTPS);
            telemetry.addData("ServoPos", position);
            telemetry.update();



            motor.setVelocity(targetTPS);


        }

    }
}
