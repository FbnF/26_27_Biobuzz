package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TestTeleOp")
public class TestTeleop extends OpMode {

    private Follower follower;

    private float turretSpeed = 1325;
    private double intakeSpeed = 1.0;
    private double turretServoAngle = 0.5;
    private DcMotorEx launchMotor;
    private DcMotor intakeMotor;
    private Servo turretServo;

    private  boolean isSpinning = false;

    private CRServo spindexServo;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        spindexServo = hardwareMap.get(CRServo.class, "spindexerServo");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretServo.setPosition(0.0);
    }

    @Override

    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        if(gamepad1.leftTriggerWasPressed()){
            turretSpeed += 50;
        }
        if(gamepad1.rightTriggerWasPressed()){
            turretSpeed -= 50;
        }
        if(gamepad1.aWasPressed()){
            launchMotor.setVelocity(turretSpeed);
        }
        if(gamepad1.bWasPressed()){
            intakeMotor.setPower(intakeSpeed);
        }
        if(gamepad1.xWasPressed()){
            turretServo.setPosition(turretServoAngle);
        }
        if(gamepad1.leftBumperWasPressed()){
            turretServoAngle -= 0.05;
        }
        if(gamepad1.rightBumperWasPressed()) {
            turretServoAngle += 0.05;
        }
        if(gamepad1.yWasPressed()){
            isSpinning = !isSpinning;
            if(isSpinning){
                spindexServo.setPower(1);
            } else{
                spindexServo.setPower(0);
            }
        }
        turretServo.setPosition(turretServoAngle);


        telemetry.addData("Turret Velocity", turretSpeed);
        telemetry.update();
        follower.update();
    }
}