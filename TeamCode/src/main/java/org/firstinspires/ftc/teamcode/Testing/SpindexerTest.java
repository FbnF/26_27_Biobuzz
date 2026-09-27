package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;


@TeleOp(name = "SpindexerTest", group = "Testing")
public class SpindexerTest extends LinearOpMode {

    private Servo servo;
    private NormalizedColorSensor colorSensor;
    double targetPos;
    boolean yellow_r = false;
    boolean yellow_g = false;
    boolean yellow_b = false;
    boolean yellow_a = false;
    NormalizedRGBA colors;
    int[] filled = new int[4];
    int currentSpot = 0;// no longer initialized here


    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        telemetry.addLine("COLORSENSOR: READY");
        telemetry.update();
        servo = hardwareMap.get(Servo.class, "servo");
        PwmControl.PwmRange axonRange = new PwmControl.PwmRange(500, 2500);
        ((PwmControl) servo).setPwmRange(axonRange);
        servo.setPosition(0.0);
        colorSensor.setGain(2);
        waitForStart();

        while (opModeIsActive()) {
            colors = colorSensor.getNormalizedColors(); // read fresh values every loop

            teleUpdate();
            telemetry.update();
            if (yellow() && filled[currentSpot] == 0 && servo.getPosition() == 18*(currentSpot)){
                targetPos = degreesToPosition(18*(currentSpot+1));
                filled[currentSpot] = 1;
                currentSpot = (currentSpot + 1) % 3;

            }
            if (gamepad1.bWasPressed()){
                targetPos = 0;
                Arrays.fill(filled, 0);
            }

            servo.setPosition(targetPos);
        }
    }

    private static final double SERVO_MAX_DEGREES = 355.0;

    private double degreesToPosition(double degrees) {
        return degrees / SERVO_MAX_DEGREES;
    }
    private boolean yellow(){
        yellow_r = .050 < colors.red && colors.red < .200;
        yellow_g = .060 < colors.green && colors.green < .260;
        yellow_b = .020 < colors.blue && colors.blue < .077;
        yellow_a = .950 < colors.alpha && colors.alpha < 1.0;
        return yellow_a && yellow_r && yellow_g && yellow_b;

    }
    private void teleUpdate(){
        telemetry.addData("Red: ","%.3f",colors.red);
        telemetry.addData("| ", yellow_r);
        telemetry.addLine();
        telemetry.addData("Green: ","%.3f",colors.green);
        telemetry.addData("| ", yellow_g);
        telemetry.addLine();
        telemetry.addData("Blue: ","%.3f",colors.blue);
        telemetry.addData("| ", yellow_b);
        telemetry.addLine();
        telemetry.addData("Alpha: ","%.3f",colors.alpha);
        telemetry.addData("| ", yellow_a);
    }

}