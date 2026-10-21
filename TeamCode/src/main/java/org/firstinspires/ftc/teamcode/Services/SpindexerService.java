package org.firstinspires.ftc.teamcode.Services;

import static android.graphics.Color.argb;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Config.SpindexerConfig;

public final class SpindexerService {
    public void initializeSpindexer(DcMotorEx SpindexerMotor, ColorSensor ColorSensor){
        SpindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void revolve(int place, DcMotorEx SpindexerMotor, ColorSensor ColorSensor){
        SpindexerMotor.setPower(1);
        SpindexerMotor.setTargetPosition(SpindexerConfig.ticksPerSlot * place);
      /*  if(ColorSensor.argb() == argb(1,255, 255, 0)){

        }*/
        if(ColorSensor.red() >= 100 && ColorSensor.green() >= 100){
            SpindexerConfig.currentLoad.set(place, 1);
        } else if(ColorSensor.red() >= 200 && ColorSensor.green() <= 100){
            SpindexerConfig.currentLoad.set(place, 3);
        } else if(ColorSensor.blue() >= 200 && ColorSensor.green() <= 100){
            SpindexerConfig.currentLoad.set(place, 4);
        } else{
            SpindexerConfig.currentLoad.set(place, 0);
        }
    }

}
