package org.firstinspires.ftc.teamcode.Services;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Localization {
    private static Pose superPose(Limelight3A limelight3A, Follower follower, GoBildaPinpointDriver pinpoint) {
        final PoseFactory p = PoseFactory.degrees();

        // 1. Update robot orientation from the Pinpoint driver for MegaTag2
        double robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
        limelight3A.updateRobotOrientation(robotYaw);

        // 2. Get current dead-reckoning pose from the follower
        double currentX = follower.pose().x();
        double currentY = follower.pose().y();
        double currentHeading = follower.pose().heading();

        // 3. Check for valid Limelight vision data
        LLResult result = limelight3A.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double visionX = botpose_mt2.getPosition().x;
                double visionY = botpose_mt2.getPosition().y;
                double visionHeading = botpose_mt2.getOrientation().getYaw();

                /* Complementary Kalman Filter
                Simple filter that smooths jumpy data
                Update alpha to update how much the limelight's MT2 should be "trusted"
                alpha determines how much weight is given to vision vs odometry.
                alpha = 1.0 means trust vision completely. alpha = 0.0 means trust odometry completely.
                The lower the value, the smoother the change.
                 */

                double alpha = 0.15;
                currentX = currentX + alpha * (visionX - currentX);
                currentY = currentY + alpha * (visionY - currentY);
                currentHeading = currentHeading + alpha * (visionHeading - currentHeading);
            }
        }

        // 4. Update the follower with the filtered pose and return it
        Pose finalPose = p.of(currentX, currentY, currentHeading);
        follower.setPose(finalPose);
        return finalPose;
    }

}

