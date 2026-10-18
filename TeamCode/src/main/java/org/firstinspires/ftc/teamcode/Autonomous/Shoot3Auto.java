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

@Autonomous(name = "Shoot 3 Autonomous", group = "Autonomous")
public class Shoot3Auto extends OpMode {
    private Follower follower;
    private Limelight3A limeLight;
    private DcMotorEx launchMotor;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose parkPose = p.of(0, 0, 90);
    private final Pose shootPose3 = p.of(10.761326860841436, 129.8657463592233, 295);

    private Path shoot3Path() {
        return line(startPose, shootPose3).linear(startPose, shootPose3);
    }

    private Path parkPath() {
        return line(shootPose3, parkPose).linear(shootPose3, parkPose);
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
        schedule(follow(follower, shoot3Path()));
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
        telemetry.addData("Status", "Executing Shoot 3 Auto");
        telemetry.update();
    }
}
