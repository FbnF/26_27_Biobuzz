package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "FakeAuto", group = "Autonomous")
public class FakeAuto extends LinearOpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final PoseFactory p = PoseFactory.degrees();

    // field poses
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose midPose = p.of(48, 72, 90);
    private final Pose parkPose = p.of(72, 36, 180);

    // passive vision state
    private Pose lastLimelightPose = null;
    private boolean hasLimelightReading = false;

    private Path startToMid() {
        return line(startPose, midPose).linear(startPose, midPose);
    }

    private Path midToPark() {
        return line(midPose, parkPose).linear(midPose, parkPose);
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
        } catch (Exception ignored) {
            limelight = null;
        }

        telemetry.addLine("fake auto ready");
        telemetry.addData("limelight", limelight != null ? "ok" : "missing");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        int state = 0;
        follower.follow(startToMid());

        try {
            while (opModeIsActive()) {
                follower.update();

                // passive mt2 reading without blocking drive loop
                if (limelight != null) {
                    try {
                        double robotYawDeg = Math.toDegrees(follower.pose().heading());
                        limelight.updateRobotOrientation(robotYawDeg);

                        LLResult result = limelight.getLatestResult();
                        if (result != null && result.isValid()) {
                            Pose3D mt2 = result.getBotpose_MT2();
                            if (mt2 != null) {
                                double px = mt2.getPosition().y + 72.0;
                                double py = 72.0 - mt2.getPosition().x;
                                double phRad = Math.toRadians(mt2.getOrientation().getYaw() + 90.0);

                                lastLimelightPose = p.of(px, py, Math.toDegrees(phRad));
                                hasLimelightReading = true;
                            }
                        }
                    } catch (Exception ignored) {}
                }

                // path sequence
                if (state == 0 && !follower.isBusy()) {
                    follower.follow(midToPark());
                    state = 1;
                } else if (state == 1 && !follower.isBusy()) {
                    state = 2;
                }

                telemetry.addLine("fake auto running");
                telemetry.addData("state", state);
                telemetry.addData("odom x", "%.2f", follower.pose().x());
                telemetry.addData("odom y", "%.2f", follower.pose().y());
                telemetry.addData("odom head", "%.1f deg", Math.toDegrees(follower.pose().heading()));
                telemetry.addData("limelight mt2", hasLimelightReading && lastLimelightPose != null ?
                        String.format("(%.1f, %.1f)", lastLimelightPose.x(), lastLimelightPose.y()) : "none");
                telemetry.update();

                if (state == 2) break;
            }
        } finally {
            computeAndSaveWeightedEndPose();
        }
    }

    private void computeAndSaveWeightedEndPose() {
        Pose odomPose = follower.pose();

        double wTarget = 0.50;
        double wLimelight = hasLimelightReading ? 0.35 : 0.0;
        double wOdom = hasLimelightReading ? 0.15 : 0.50;

        double finalX = wTarget * parkPose.x() + wOdom * odomPose.x();
        double finalY = wTarget * parkPose.y() + wOdom * odomPose.y();
        double finalHeading = wTarget * parkPose.heading() + wOdom * odomPose.heading();

        if (hasLimelightReading && lastLimelightPose != null) {
            finalX += wLimelight * lastLimelightPose.x();
            finalY += wLimelight * lastLimelightPose.y();
            finalHeading += wLimelight * lastLimelightPose.heading();
        }

        Pose weightedEndPose = p.of(finalX, finalY, Math.toDegrees(finalHeading));
        OpModeStorage.saveAutoEndPose(weightedEndPose);

        telemetry.addLine("auto end pose saved");
        telemetry.addData("saved x", "%.2f", weightedEndPose.x());
        telemetry.addData("saved y", "%.2f", weightedEndPose.y());
        telemetry.addData("saved head", "%.1f deg", Math.toDegrees(weightedEndPose.heading()));
        telemetry.update();
    }
}
