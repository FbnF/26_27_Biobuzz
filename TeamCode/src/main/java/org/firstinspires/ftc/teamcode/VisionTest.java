package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@TeleOp(name = "VisionTestTeleop")
public class VisionTest extends OpMode {


  //  private DcMotorEx launchMotor;
  //  private DcMotor intakeMotor;
  //  private Servo turretServo;


  //  private Servo spindexServo;

    private OpenCvWebcam openCvCam;

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound = 0;
    double maxContourArea = 0;

    public enum Stage {
        YCbCr_CHAN2,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE,
    }

    private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
    private Stage[] stages = Stage.values();

    public void onViewportTapped() {
        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
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
            double largestArea = 0;
            for (MatOfPoint contour : contoursList) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                }
            }
            maxContourArea = largestArea;

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
       // launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
     //   intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
      //  turretServo = hardwareMap.get(Servo.class, "turretServo");
      //  spindexServo = hardwareMap.get(Servo.class, "spindexerServo");
     //   intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    //    launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     //   turretServo.setPosition(0.0);

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
    public void loop() {

        boolean ballDetected = numContoursFound > 0 && maxContourArea > 100;

        telemetry.addData("Ball Detected", ballDetected ? "YES" : "NO");
        telemetry.addData("Contours Found", numContoursFound);
        telemetry.addData("Max Blob Area", "%.1f", maxContourArea);
        telemetry.update();

    }
}