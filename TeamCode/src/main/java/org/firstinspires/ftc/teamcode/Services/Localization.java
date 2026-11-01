package org.firstinspires.ftc.teamcode.Services;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@Config

public class Localization {

    private static Pose superPose(Limelight3A limelight3A, Follower follower, GoBildaPinpointDriver pinpoint, double alpha) {
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
                double visionX = MT2ToPedro(botpose_mt2).x();
                double visionY = MT2ToPedro(botpose_mt2).y();
                double visionHeading = MT2ToPedro(botpose_mt2).heading();

                /* Complementary Kalman Filter
                Simple filter that smooths jumpy data
                 */

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

    private static Pose MT2ToPedro(@org.jetbrains.annotations.UnknownNullability Pose3D MegaTag2){
        double Px = MegaTag2.getPosition().y + 72;
        double Py = 72 - MegaTag2.getPosition().x;
        double Ph = MegaTag2.getOrientation().getYaw() + 90;
        return PoseFactory.degrees().of(Px, Py, Ph);
    }
}

