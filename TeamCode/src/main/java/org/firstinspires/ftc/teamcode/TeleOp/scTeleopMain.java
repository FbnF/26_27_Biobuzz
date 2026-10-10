package org.firstinspires.ftc.teamcode.Sensors;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "scTeleopMain", group = "")
public class Motor extends LinearOpMode {
    DcMotor intakeMotor;
    DcMotor launchMotor;
    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor")
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a == true){
                intakeMotor.setPower(0.5);
        }
            if(gamepad1.b == true){
                launchMotor.setPower(0.5)
            }
    }
}

