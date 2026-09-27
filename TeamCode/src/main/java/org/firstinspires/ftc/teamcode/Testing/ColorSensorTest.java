package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ColorSensorTest", group = "Testing")
public class ColorSensorTest extends LinearOpMode {
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        telemetry.addLine("COLORSENSOR: READY");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();

            telemetry.addLine("RGBA VALUES");
            telemetry.addData("RED: ", red);
            telemetry.addData("GREEN: ", green);
            telemetry.addData("BLUE: ", blue);
            telemetry.addData("ALPHA: ", alpha);
        }
    }
}
