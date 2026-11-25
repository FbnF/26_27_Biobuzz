package org.firstinspires.ftc.teamcode.Services;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.ShooterConfig;

import java.util.List;

// Vision logic
public class LLService {

    private Limelight3A limelight;
    private double filteredDistance = 0.0;
    private double rawDistance = 0.0;
    private boolean targetVisible = false;
    private boolean hasGoalTag = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private int currentTagId = -1;
    private int[] goalTagIds = new int[]{24};
    private boolean isBlue = false;
    private boolean visionEnabled = true;

    private long lastSeenNs = 0;
    private static final long NULL_DELAY_NS = 2_000_000_000L;

    private boolean currentlySeeing = false;

    private static VisionService instance = null;

    public static VisionService getInstance() {
        if (instance == null) instance = new VisionService();
        return instance;
    }

    public void setGoalTagIds(int... ids) { this.goalTagIds = ids != null ? ids : new int[0]; }
    public void setGoalTagId(int id) { this.goalTagIds = new int[]{id}; }
    public void setIsBlue(boolean blue) { this.isBlue = blue; }
    public void setVisionEnabled(boolean enabled) { this.visionEnabled = enabled; }
    public boolean isVisionEnabled() { return visionEnabled; }

    public void reset() {
        filteredDistance = 0.0;
        rawDistance = 0.0;
        tx = 0.0;
        ty = 0.0;
        currentTagId = -1;
        lastSeenNs = 0;
        currentlySeeing = false;
        targetVisible = false;
        hasGoalTag = false;
        visionEnabled = true;
    }

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, false);
    }

    public void init(HardwareMap hardwareMap, boolean isRed) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        if(isRed){
            limelight.pipelineSwitch(3);
        }else{
            limelight.pipelineSwitch(2);
        }

    }

    public Limelight3A returnLimelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        return limelight;
    }

    public int getPipeline(LLResult result){
        return result.getPipelineIndex();
    }


    public void update() {
        if (!visionEnabled || limelight == null) {
            targetVisible = false;
            hasGoalTag = false;
            return;
        }

        LLResult result = limelight.getLatestResult();
        currentlySeeing = false;
        hasGoalTag = false;

        if (result != null && result.isValid() && result.getStaleness() < 250) {
            if(result.getPipelineIndex() != 2){
                limelight.pipelineSwitch(2);
                return;
            }
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (f != null) {
                        boolean matches = false;
                        for (int id : goalTagIds) {
                            if (f.getFiducialId() == id) {
                                matches = true;
                                break;
                            }
                        }
                        if (matches) {
                            Pose3D pose = f.getCameraPoseTargetSpace();
                            if (pose != null) {
                                rawDistance = Math.sqrt(Math.pow(pose.getPosition().x, 2) + Math.pow(pose.getPosition().z, 2)) * 39.3701;

                                // Smoothing
                                if (filteredDistance <= 0.0) filteredDistance = rawDistance;
                                double a = ShooterConfig.DIST_SMOOTH_ALPHA;
                                filteredDistance = (1.0 - a) * filteredDistance + a * rawDistance;

                                currentTagId = f.getFiducialId();
                                tx = f.getTargetXDegrees();
                                ty = f.getTargetYDegrees();
                                currentlySeeing = true;
                                hasGoalTag = true;
                                lastSeenNs = System.nanoTime();
                                break;
                            }
                        }
                    }
                }
            }
        }

        // Null delay buffer
        if (currentlySeeing) {
            targetVisible = true;
        } else {
            targetVisible = (System.nanoTime() - lastSeenNs) < NULL_DELAY_NS;
        }
    }

    public boolean isNoShotZone() {
        return targetVisible && ShooterConfig.NO_SHOT_UNDER_IN > 0.0 && filteredDistance < ShooterConfig.NO_SHOT_UNDER_IN;
    }

    public double[] getTxWindow() {
        if (ShooterConfig.isGroupB(currentTagId)) {
            return interpolateWindow(filteredDistance, ShooterConfig.DIST_IN_GROUP_B, ShooterConfig.MIN_ANGLE_GROUP_B, ShooterConfig.MAX_ANGLE_GROUP_B);
        } else {
            return interpolateWindow(filteredDistance, ShooterConfig.DIST_IN_GROUP_A, ShooterConfig.MIN_ANGLE_GROUP_A, ShooterConfig.MAX_ANGLE_GROUP_A);
        }
    }

    private double[] interpolateWindow(double x, double[] dists, double[] mins, double[] maxs) {
        if (dists == null || mins == null || maxs == null || dists.length < 2) return new double[]{-5, 5};

        int i = 0;
        int last = dists.length - 1;
        if (x <= dists[0]) i = 0;
        else if (x >= dists[last]) i = last - 1;
        else {
            while (i < last - 1 && x > dists[i + 1]) i++;
        }

        double x0 = dists[i];
        double x1 = dists[i+1];
        double t = (x - x0) / (x1 - x0);

        return new double[]{
                mins[i] + t * (mins[i+1] - mins[i]),
                maxs[i] + t * (maxs[i+1] - maxs[i])
        };
    }

    public double getDistance() { return filteredDistance; }
    public double getRawDistance() { return rawDistance; }
    public boolean isTargetVisible() { return targetVisible; }
    public boolean hasGoalTag() { return hasGoalTag; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public boolean isCurrentlySeeing() { return currentlySeeing; }
    public int[] getGoalTagIds() { return goalTagIds; }
    public int getCurrentTagId() { return currentTagId; }
}
