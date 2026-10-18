package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Shoot 1 Autonomous", group = "Autonomous")
public class Shoot1Auto extends OpMode {
    private Follower follower;
    private Limelight3A limeLight;
    private DcMotorEx launchMotor;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose parkPose = p.of(0, 0, 90);
    private final Pose shootPose1 = p.of(57.18932038834951, 23.653721682847912, 90);

    private Path shoot1Path() {
        return line(startPose, shootPose1).linear(startPose, shootPose1);
    }

    private Path parkPath() {
        return line(shootPose1, parkPose).linear(shootPose1, parkPose);
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");

        limeLight = hardwareMap.get(Limelight3A.class, "limelight");
        limeLight.setPollRateHz(100);
        limeLight.start();
        limeLight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        schedule(follow(follower, shoot1Path()));
    }

    @Override
    public void loop() {
        LLResult result = limeLight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.ColorResult> colorBlobs = result.getColorResults();
            for (LLResultTypes.ColorResult blob : colorBlobs) {
                telemetry.addData("Target X", blob.getTargetXDegrees());
                telemetry.addData("Target Y", blob.getTargetYDegrees());
            }
        }
        telemetry.addData("Status", "Executing Shoot 1 Auto");
        telemetry.update();
    }
}
