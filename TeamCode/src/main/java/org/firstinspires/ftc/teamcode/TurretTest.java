package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TurretTest", group = "Turret")

public class TurretTest extends LinearOpMode {
    DcMotorEx motor;
    double TPS = 1000;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "launchMotor");
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
            motor.setVelocity(TPS);
            telemetry.addData("TPS:" ,TPS);
            telemetry.update();
        }

    }

}
