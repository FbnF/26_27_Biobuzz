package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Config.TurretConfig;

import org.opencv.core.Mat;

// Status light logic
public class TurretServiceLegacy {

    private static TurretServiceLegacy instance = null;


    public double GetRotationAngle(double targetX, double targetY){
        double angleOfHive =  Math.atan(targetX/targetY);
        if(angleOfHive  <= TurretConfig.max && angleOfHive >= TurretConfig.min){
            double Vx = Math.cos(angleOfHive) * 10;
            double Vy = Math.sin(angleOfHive) * 10;
            double V = Math.sqrt(Math.pow(Vx, 2)+Math.pow(Vy, 2));
            double X = Vx/Math.abs(V) * 2.23607;
            double Y = Vy/Math.abs(V) * 2.23607;
            return Math.atan2(X,Y);

        } else{
            return(0.0);
        }
    }

    public void FireBasedOnAprilTag(DcMotorEx launchMotor, Servo spindexServo,Servo turrentChange, double velocity, int[] ElementType,int ElementId, double turnAngle){
        //For the ElementType 0:Empty ,1: Pollen, 2: Blue Nectar, 3: Red Nectar
        if(ElementType[ElementId] == 0){
            FireBasedOnAprilTag(launchMotor, spindexServo, turrentChange, velocity,ElementType,ElementId + 1, turnAngle);
        } else if (ElementType[ElementId] == 2 || ElementType[ElementId] == 3) {
            turrentChange.setPosition(1.0);//Change this later
        } else if (ElementType[ElementId] == 1) {
            turrentChange.setPosition(0);
        }
        //The velocity will be calculated on the fly once we tune it.
        launchMotor.setVelocity(velocity);
        spindexServo.setPosition(turnAngle);
    }


}
