package org.firstinspires.ftc.teamcode.Autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Services.TurretService;
import org.firstinspires.ftc.teamcode.pedro.Constants;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueStateMachineAuto")
public class BlueStateMachineAuto extends OpMode {
    private Follower follower;
    private int state = 0;
    private int previousState = -1; // Prevents re-scheduling paths 50x a second
    // State 0 is park, State 1 is avoid, State 2 is chase, State 3 is shooter position 1, State 4 is shooter position 2, State 5 is shooter position 3

    private OpenCvWebcam openCvCam;
    private Limelight3A limeLight;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;



    private TurretService turretService;
    private DcMotorEx launchMotor;

    private boolean isJustPickedUp = false;


    private ElapsedTime autoTimer = new ElapsedTime();
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(0, 0, 90);
    private Pose currentPose = startPose;
    private final Pose controlPose = p.of(12, 12, 45);
    private final Pose shootPose = p.of(57.18932038834951, 23.653721682847912, 90);
    private final Pose shootPose2 = p.of(57.01213592233011, 117.73063309061487, 270);
    private final Pose shootPose3 = p.of(10.761326860841436, 129.8657463592233, 295);


    // OpenCV Mats (declared once to avoid memory leaks)
    Mat hsvMat = new Mat();
    Mat redMask = new Mat();
    Mat blueMask = new Mat();
    Mat yellowMask = new Mat();
    Mat hierarchy = new Mat();
    Mat contoursOnFrameMat = new Mat();

    List<MatOfPoint> PollenContoursList = new ArrayList<>();
    List<MatOfPoint> RedContoursList = new ArrayList<>();
    List<MatOfPoint> BlueContoursList = new ArrayList<>();

    volatile int numContoursFound = 0;
    volatile double maxPollenContourArea = 0;
    volatile double maxBlueContourArea = 0;
    volatile double maxRedContourArea = 0;

    // HSV Thresholds: Scalar(Hue [0-180], Saturation [0-255], Value [0-255])
    Scalar lowerRed = new Scalar(0, 120, 70);
    Scalar upperRed = new Scalar(10, 255, 255);

    Scalar lowerBlue = new Scalar(100, 150, 70);
    Scalar upperBlue = new Scalar(130, 255, 255);

    Scalar lowerYellow = new Scalar(20, 100, 100);
    Scalar upperYellow = new Scalar(35, 255, 255);

    public enum Stage {
        CONTOURS_OVERLAYED_ON_FRAME,
        RED_MASK,
        BLUE_MASK,
        YELLOW_MASK,
        RAW_IMAGE
    }

    private Stage stageToRenderToViewport = Stage.CONTOURS_OVERLAYED_ON_FRAME;
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

        private double findLargestContour(Mat mask, List<MatOfPoint> contourList) {
            for (MatOfPoint c : contourList) {
                c.release(); // Free native RAM before clearing
            }
            contourList.clear();

            Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double largestArea = 0;
            for (MatOfPoint contour : contourList) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                }
            }
            return largestArea;
        }

        @Override
        public Mat processFrame(Mat input) {
            // 1. Convert RGB input to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // 2. Create separate masks for Pollen (Yellow), Red, and Blue
            Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);
            Core.inRange(hsvMat, lowerRed, upperRed, redMask);
            Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);

            // 3. Find contours on each specific color mask
            maxPollenContourArea = findLargestContour(yellowMask, PollenContoursList);
            maxRedContourArea = findLargestContour(redMask, RedContoursList);
            maxBlueContourArea = findLargestContour(blueMask, BlueContoursList);

            numContoursFound = PollenContoursList.size() + RedContoursList.size() + BlueContoursList.size();

            // 4. Draw color-coded contours onto the preview frame
            input.copyTo(contoursOnFrameMat);
            Imgproc.drawContours(contoursOnFrameMat, PollenContoursList, -1, new Scalar(255, 255, 0), 3, 8); // Yellow
            Imgproc.drawContours(contoursOnFrameMat, RedContoursList, -1, new Scalar(255, 0, 0), 3, 8);      // Red
            Imgproc.drawContours(contoursOnFrameMat, BlueContoursList, -1, new Scalar(0, 0, 255), 3, 8);     // Blue

            switch (stageToRenderToViewport) {
                case YELLOW_MASK:
                    return yellowMask;

                case RED_MASK:
                    return redMask;

                case BLUE_MASK:
                    return blueMask;

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
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

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
        previousState = 0;
    }

    @Override
    public void loop() {
        LLResult result = limeLight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < 100) {
            telemetry.addData("LimeLight Tx", "%.2f", result.getTx());
            telemetry.addData("LimeLight Ty", "%.2f", result.getTy());
            telemetry.addData("LimeLight Ta", "%.2f", result.getTa());
        }
        currentPose = follower.pose();

        boolean pollenDetected = maxPollenContourArea > 100;
        boolean redDetected = maxRedContourArea > 100;
        boolean blueDetected = maxBlueContourArea > 100;
        boolean anyBallDetected = pollenDetected || redDetected || blueDetected;

        // Example logic: Chase Pollen or Red, Avoid Blue
        if (pollenDetected || blueDetected) {
            state = 2; // Chase
            isJustPickedUp = true;
        } else if (redDetected) {
            state = 1; // Avoid
        } else if (!anyBallDetected && isJustPickedUp) {
            state = 3; // Shoot1
            isJustPickedUp = false;
        }


        telemetry.addData("Contours Found", numContoursFound);
        telemetry.addData("Pollen (Yellow) Area", "%.1f (Detected: %b)", maxPollenContourArea, pollenDetected);
        telemetry.addData("Red Area", "%.1f (Detected: %b)", maxRedContourArea, redDetected);
        telemetry.addData("Blue Area", "%.1f (Detected: %b)", maxBlueContourArea, blueDetected);


        if(autoTimer.seconds() < 5){
            state = 0;
        }
        // Only schedule the path once when entering a new state
        if (state != previousState) {
            switch (state) {
                case 0:
                    schedule(follow(follower, park()));
                    break;
                case 1:
                    schedule(follow(follower, circle()));
                    break;
                case 2:
                    leftFront.setVelocity(1000);
                    leftBack.setVelocity(1000);
                    rightBack.setVelocity(1000);
                    rightFront.setVelocity(1000);
                    break;
                case 3:
                    schedule(follow(follower, shoot1()));
                    break;
                case 4:
                    schedule(follow(follower, shoot2()));
                    break;
                case 5:
                    schedule(follow(follower, shoot3()));
                    break;
            }
            previousState = state;
        }



        telemetry.addData("Current State", state);
        Scheduler.execute();
        telemetry.update();
        //turretService.FireBasedOnAprilTag(launchMotor, 900);
    }
}