package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;

import org.opencv.core.Mat;

// Status light logic
public class TurretService {

    private static TurretService instance = null;

    private DcMotorEx turretMotor = null;
    private int targetTicks = 0;
    private int lastDirection = 0;
    private boolean turretInitialized = false;


    public void initTurret(DcMotorEx motor){
        turretMotor = motor;
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetTicks = 0;
        lastDirection = 0;
        turretInitialized = true;
    }

    public void setTargetAngle(double radians){
        setTargetTicks((int)Math.round(radians * TurretConfig.TICKS_PER_RADIAN));
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
        if(targetX == 0.0 && targetY == 0.0){
            return TurretConfig.min;
        }
        double angleOfHive = Math.atan2(targetX, targetY);
        if(angleOfHive < TurretConfig.min){
            return TurretConfig.min;
        }
        if(angleOfHive > TurretConfig.max){
            return TurretConfig.max;
        }
        return angleOfHive;
    }

    public void FireBasedOnAprilTag(DcMotorEx launchMotor, Servo spindexServo,Servo turrentChange, double velocity, int[] ElementType,int ElementId, double turnAngle){
        //For the ElementType 0:Empty ,1: Pollen, 2: Blue Nectar, 3: Red Nectar
        if(ElementType == null || ElementId < 0 || ElementId >= ElementType.length){
            return;
        }

        int target = -1;
        for(int i = ElementId; i < ElementType.length; i++){
            if(ElementType[i] != 0){
                target = i;
                break;
            }
        }
        if(target == -1){
            return;
        }

        int elementType = ElementType[target];
        if(elementType == 2 || elementType == 3){
            turrentChange.setPosition(1.0);//Change this later
        } else if(elementType == 1){
            turrentChange.setPosition(0);
        }
        //The velocity will be calculated on the fly once we tune it.
        launchMotor.setVelocity(velocity);
        spindexServo.setPosition(turnAngle);
    }


}