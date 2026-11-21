package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Config.HardwareConfig;
import org.firstinspires.ftc.teamcode.Services.EKalmanFilterAI;

@TeleOp(name = "RedTeleop")
public class RedTeleop extends OpMode {

    private Follower follower;
    private DcMotorEx turretMotor;
    private int deltaT;

    double initialX = 131.53;
    double initialY = 23.08;
    double initialTheta = 0.0;

    private double deltaDist = 0.4;
    private double deltaTheta = 0.005;

    private boolean tagDetected = false;
    private double tagX = 0.0;
    private double tagY = 0.0;
    private double tagHeading = 0.0;



    private EKalmanFilterAI ekf = new EKalmanFilterAI(initialX, initialY, initialTheta);




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

        deltaDist += Math.hypot(follower.pose().x(),follower.pose().y()) ;
        deltaTheta = follower.pose().heading();


        double[] fusedPose = ekf.update(deltaDist, deltaTheta, tagDetected, tagX, tagY, tagHeading);

        deltaT += 1;

        follower.update();
        telemetry.addData("Elapsed Time in Ms", deltaT * 20);
        telemetry.addData("Kalman X", fusedPose[0]);
        telemetry.addData("Kalman Y", fusedPose[1]);
        telemetry.addData("Kalman theta", fusedPose[2]);
        telemetry.update();
    }
}