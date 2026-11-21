package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Services.EKalmanFilterAI;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Kalman Tuning")
public class KalmanTuning extends OpMode {

    private Follower follower;
    private DcMotorEx turretMotor;

    private int deltaT;

    private final double initialX = 0;
    private final double initialY = 0;
    private final double initialTheta = 0.0;

    private double deltaDist = 0.4;
    private double deltaTheta = 0.005;

    private boolean tagDetected = false;
    private double tagX = 0.0;
    private double tagY = 0.0;
    private double tagHeading = 0.0;

    private double prevX = initialX;
    private double prevY = initialY;
    private double prevTheta = initialTheta;

    private final EKalmanFilterAI ekf = new EKalmanFilterAI(initialX, initialY, initialTheta);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(initialX, initialY, initialTheta));
        //  turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
    }

    @Override
    public void loop() {
        follower.update();

        // Manual drive - Pedro 2 API (same pattern as TeleOpMainBlue)
        double forward = -gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        follower.manual(forward, lateral, turn);


        Pose pose = follower.pose();

        double dx = pose.x() - prevX;
        double dy = pose.y() - prevY;

        double currentTheta = pose.heading();
        deltaTheta = currentTheta - prevTheta;
        while (deltaTheta > Math.PI) deltaTheta -= 2.0 * Math.PI;
        while (deltaTheta < -Math.PI) deltaTheta += 2.0 * Math.PI;

        double avgTheta = prevTheta + (deltaTheta / 2.0);
        deltaDist = dx * Math.cos(avgTheta) + dy * Math.sin(avgTheta);

        prevX = pose.x();
        prevY = pose.y();
        prevTheta = currentTheta;

        double[] fusedPose = ekf.update(deltaDist, deltaTheta, tagDetected, tagX, tagY, tagHeading);

        deltaT++;

        telemetry.addData("Elapsed Time in Ms", deltaT * 20);
        telemetry.addData("Kalman X", fusedPose[0]);
        telemetry.addData("Kalman Y", fusedPose[1]);
        telemetry.addData("Kalman theta", fusedPose[2]);
        telemetry.update();
    }
}
