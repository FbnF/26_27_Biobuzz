package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.Services.TurretService;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.teamcode.Services.VisionService;
import org.firstinspires.ftc.teamcode.Config.VisionConfig;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class TestAutonomous extends OpMode {
    private Follower follower;
    private int state = 0;
    // State 0 is park, State 1 is avoid, State 2 is chase, State 3 is shooter position 1, State 4 is shooter position 2, State 5 is shooter position 3

    private OpenCvWebcam openCvCam;
    private Limelight3A limeLight;

    private TurretService turretService;

    private DcMotorEx launchMotor;

    private boolean isJustPickedUp = false;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(0, 0, 90);
    private final Pose currentPose = startPose;
    private final Pose controlPose = p.of(12, 12, 45);
    private final Pose shootPose = p.of(57.18932038834951, 23.653721682847912, 90);
    private final Pose shootPose2 = p.of(57.01213592233011, 117.73063309061487, 270);
    private final Pose shootPose3 = p.of(10.761326860841436, 129.8657463592233, 295);

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound;

    public enum Stage {
        YCbCr_CHAN2,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE
    }

    private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
    private final Stage[] stages = Stage.values();

    public void onViewportTapped() {
        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    private Path park() {
        return line(currentPose, park).linear(startPose, park);
    }

    private Path shoot1() {
        return line(currentPose, shootPose).linear(startPose, shootPose);
    }

    private Path shoot2() {
        return line(currentPose, shootPose2).linear(startPose, shootPose2);
    }

    private Path shoot3() {
        return line(currentPose, shootPose3).linear(startPose, shootPose3);
    }

    private Path shoot1To2() {
        return line(shootPose, shootPose2).linear(shootPose, shootPose2);
    }

    private Path shoot2To3() {
        return line(shootPose2, shootPose3).linear(shootPose2, shootPose3);
    }

    private Path shoot1ToPark() {
        return line(shootPose, park).linear(shootPose, park);
    }

    private Path shoot2ToPark() {
        return line(shootPose2, park).linear(shootPose2, park);
    }

    private Path shoot3ToPark() {
        return line(shootPose3, park).linear(shootPose3, park);
    }

    private Path circle() {
        return curve(startPose, controlPose, park).linear(startPose, park);
    }

    private Path curvedShoot1() {
        return curve(currentPose, controlPose, shootPose).linear(startPose, shootPose);
    }

    private Path curvedShoot2() {
        return curve(currentPose, controlPose, shootPose2).linear(startPose, shootPose2);
    }

    private Path curvedShoot3() {
        return curve(currentPose, controlPose, shootPose3).linear(startPose, shootPose3);
    }

    public class VisionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);

            switch (stageToRenderToViewport) {
                case YCbCr_CHAN2:
                    return yCbCrChan2Mat;

                case THRESHOLD:
                    return thresholdMat;

                case CONTOURS_OVERLAYED_ON_FRAME:
                    return contoursOnFrameMat;

                case RAW_IMAGE:
                default:
                    return input;
            }
        }
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "OpenCVCam");
        openCvCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        openCvCam.setPipeline(new VisionPipeline());
        openCvCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                openCvCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    @Override
    public void start() {
        schedule(follow(follower, park()));
    }

    @Override
    public void loop() {
        //TODO Change this to work with april tags
            LLResult ll = null;
            LLResult tmp = limeLight.getLatestResult();
            if (tmp != null && tmp.isValid() && tmp.getStaleness() < 100) {
                ll = tmp;
            }



        boolean ballDetected = numContoursFound > 0 ;
        if(ballDetected){
            state = 2;
            isJustPickedUp = true;
        }

        if(!ballDetected && isJustPickedUp){
            state = 3;
        }

        telemetry.addData("Contours Found", numContoursFound);

        switch (state) {
            case 0:
                schedule(follow(follower, park()));
                telemetry.addData("State: ", "Park");
                break;
            case 1:
                schedule(follow(follower, circle()));
                telemetry.addData("State: ", "Avoid");
                break;
            case 2:
                telemetry.addData("State: ", "Chase");
                break;
            case 3:
                schedule(follow(follower, shoot1()));
                telemetry.addData("State: ", "Shoot1");
                break;
            case 4:
                schedule(follow(follower, shoot2()));
                telemetry.addData("State: ", "Shoot2");
                break;
            case 5:
                schedule(follow(follower, shoot3()));
                telemetry.addData("State: ", "Shoot3");
                break;
        }

        telemetry.update();
        //turretService.FireBasedOnAprilTag(launchMotor, 900);
    }
}