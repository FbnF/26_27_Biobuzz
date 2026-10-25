package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "RedTeleop")
public class RedTeleop extends OpMode {

    private Follower follower;
    private DcMotorEx turretMotor;



    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");


    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
    }
}