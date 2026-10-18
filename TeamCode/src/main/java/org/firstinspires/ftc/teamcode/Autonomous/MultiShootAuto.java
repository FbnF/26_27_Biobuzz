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

@Autonomous(name = "Multi-Shoot Autonomous", group = "Autonomous")
public class MultiShootAuto extends OpMode {
    private Follower follower;
    private Limelight3A limeLight;
    private DcMotorEx launchMotor;

    private int state = 0;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose parkPose = p.of(0, 0, 90);
    private final Pose shootPose1 = p.of(57.18932038834951, 23.653721682847912, 90);
    private final Pose shootPose2 = p.of(57.01213592233011, 117.73063309061487, 270);
    private final Pose shootPose3 = p.of(10.761326860841436, 129.8657463592233, 295);

    private Path startToShoot1() {
        return line(startPose, shootPose1).linear(startPose, shootPose1);
    }

    private Path shoot1ToShoot2() {
        return line(shootPose1, shootPose2).linear(shootPose1, shootPose2);
    }

    private Path shoot2ToShoot3() {
        return line(shootPose2, shootPose3).linear(shootPose2, shootPose3);
    }

    private Path shoot3ToPark() {
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
        state = 1;
        schedule(follow(follower, startToShoot1()));
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

        switch (state) {
            case 1:
                telemetry.addData("Phase", "Navigating to Shoot Position 1");
                break;
            case 2:
                telemetry.addData("Phase", "Navigating to Shoot Position 2");
                break;
            case 3:
                telemetry.addData("Phase", "Navigating to Shoot Position 3");
                break;
            case 4:
                telemetry.addData("Phase", "Parking");
                break;
        }
        telemetry.update();
    }
}
