package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TurretTest", group = "Turret")

public class TurretTest extends LinearOpMode {
    DcMotorEx motor;
    Servo servo;
    double TPS = 1000;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        servo = hardwareMap.get(Servo.class, "turretServo");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.leftTriggerWasPressed()){
                TPS += 50;
            }
            if(gamepad1.rightTriggerWasPressed()){
                TPS -= 50;
            }
            if(TPS >= 2100){
                TPS = 2000;
            }
            if(gamepad1.a){
                servo.setPosition(0);
            }

            if(gamepad1.b){
                servo.setPosition(1);
            }

            motor.setVelocity(TPS);
            telemetry.addData("TPS:" ,TPS);
            telemetry.update();
        }

    }

}
