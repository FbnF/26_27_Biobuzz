package org.firstinspires.ftc.teamcode.Services;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.HardwareConfig;

public class initializationService {
    public void initialize(HardwareMap hardwareMap, DcMotorEx turretMotor, DcMotorEx turretServoMotor, DcMotorEx intakeMotor, DcMotorEx transferMotor,DcMotorEx baseTurretEncoder,Servo gateServo){
    turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
    turretServoMotor = hardwareMap.get(DcMotorEx.class, "turretServoMotor");
    intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
    gateServo = hardwareMap.get(Servo.class, "gateServo");

    }
}
