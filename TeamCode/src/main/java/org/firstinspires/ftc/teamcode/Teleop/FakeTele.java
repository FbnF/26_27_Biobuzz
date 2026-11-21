package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.Services.LoggingService;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "FakeTele", group = "TeleOp")
public class FakeTele extends LinearOpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private LoggingService logger = LoggingService.getInstance();

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);

        telemetry.addLine("fake tele ready");
        telemetry.addData("pose fresh", OpModeStorage.isPoseFresh);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // load auto end pose or fallback to origin
        Pose fallbackPose = p.of(0, 0, 0);
        Pose startPose = OpModeStorage.getPoseForTeleOp(fallbackPose);

        follower.setPose(startPose);
        logger.init(hardwareMap, follower::pose);

        try {
            while (opModeIsActive()) {
                double forward = -gamepad1.left_stick_y;
                double lateral = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;

                follower.manual(forward, lateral, turn);
                follower.update();

                logger.record();

                telemetry.addLine("fake tele running");
                telemetry.addData("x", "%.2f in", follower.pose().x());
                telemetry.addData("y", "%.2f in", follower.pose().y());
                telemetry.addData("head", "%.1f deg", Math.toDegrees(follower.pose().heading()));
                telemetry.update();
            }
        } finally {
            logger.stop();
        }
    }
}
