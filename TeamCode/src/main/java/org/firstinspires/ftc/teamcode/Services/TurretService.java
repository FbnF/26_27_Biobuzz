package org.firstinspires.ftc.teamcode.Services;

import static org.firstinspires.ftc.teamcode.Config.TurretConfig.TICKS_PER_STEP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;

import org.opencv.core.Mat;

import java.util.concurrent.CopyOnWriteArraySet;

// Status light logic
public class TurretService {


    //Desmos calcin
    //https://www.desmos.com/calculator/zo74h63iup

    private static TurretService instance = null;

    private DcMotorEx turretMotor;
    private DcMotorEx turretServoMotor;
    private int targetTicks = 0;
    private int lastDirection = 0;
    private boolean turretInitialized = false;

    private DcMotorEx encoderMotor = turretMotor;

    private double gearRatio = (16/38);

    private double absPosition = 0;

    private double targetRotationTicks;

    private double absTargetPose;

    private double oppositeRadians;

    private double turnFactor;




    public void initTurret(DcMotorEx motor, DcMotorEx ServoMotor){
        turretMotor = motor;
        if (turretMotor != null) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        turretServoMotor = ServoMotor;
        if (turretServoMotor != null) {
            turretServoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        targetTicks = 0;
        lastDirection = 0;
        turretInitialized = true;
    }

    public void initTurret(DcMotorEx motor) {
        initTurret(motor, null);
    }

    public double getNewPosition(double positionChange){
        absPosition = absPosition - positionChange;
        return absPosition;
    }

    public void setTargetAngle(double radians){
        //   setTargetTicks((int)Math.round(radians * TurretConfig.TICKS_PER_RADIAN));
        targetRotationTicks = radians * TurretConfig.TICKS_PER_RADIAN;
        oppositeRadians = radians + Math.toRadians(180);
        absTargetPose = absPosition+targetRotationTicks;

        if(absPosition == absTargetPose){
            return;
        }
        if(targetRotationTicks >= oppositeRadians){
            turnFactor = -1;
        } else{
            turnFactor = 1;
        }


        turretServoMotor.setVelocity(TICKS_PER_STEP * turnFactor);


    }

    public void setTargetTicks(int ticks){
        int minTicks = (int)Math.round(TurretConfig.min * TurretConfig.TICKS_PER_RADIAN);
        int maxTicks = (int)Math.round(TurretConfig.max * TurretConfig.TICKS_PER_RADIAN);
        if(ticks < minTicks){
            targetTicks = minTicks;
        } else if(ticks > maxTicks){
            targetTicks = maxTicks;
        } else {
            targetTicks = ticks;
        }
    }

    public boolean atTarget(){
        if(turretMotor == null){
            return false;
        }
        return Math.abs(targetTicks - turretMotor.getCurrentPosition()) <= TurretConfig.BANG_BANG_DEADBAND;
    }

    public void updateTurret(){
        if(turretMotor == null){
            return;
        }
        int error = targetTicks - turretMotor.getCurrentPosition();
        double absError = Math.abs(error);

        if(absError <= TurretConfig.BANG_BANG_DEADBAND){
            turretMotor.setPower(0);
            lastDirection = 0;
            return;
        }

        int desiredDirection = error > 0 ? 1 : -1;
        if(desiredDirection != lastDirection && absError < TurretConfig.BANG_BANG_DEADBAND + TurretConfig.BANG_BANG_HYSTERESIS){
            turretMotor.setPower(0);
        } else {
            lastDirection = desiredDirection;
            turretMotor.setPower(desiredDirection * TurretConfig.BANG_BANG_POWER);
        }
    }

    public void stopTurret(){
        if(turretMotor != null){
            turretMotor.setPower(0);
        }
        lastDirection = 0;
    }

    public int getTargetTicks(){
        return targetTicks;
    }

    public int getCurrentTicks(){
        return turretMotor != null ? turretMotor.getCurrentPosition() : 0;
    }

    public boolean isTurretInitialized(){
        return turretInitialized;
    }


    public double GetRotationAngle(double targetX, double targetY){
        double itargetX = -targetX;
        if(itargetX == 0.0 && targetY == 0.0){
            return TurretConfig.min;
        }
        double angleOfHive = Math.atan2(itargetX, targetY);
        if(angleOfHive < TurretConfig.min){
            return TurretConfig.min;
        }
        if(angleOfHive > TurretConfig.max){
            return TurretConfig.max;
        }
        return angleOfHive;
    }
/*
    public void FireBasedOnAprilTag(DcMotorEx launchMotor, DcMotorEx turrentChange, double velocity, double turnAngle, DcMotorEx transferMotor){
        if(target == -1){
            return;
        }

        //The velocity will be calculated on the fly once we tune it.
        launchMotor.setVelocity(velocity);
     //   spindexServo.setPosition(turnAngle);
    }*/


}