package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "ManualTeleop", group = "teleop")
public class ManualTeleop extends LinearOpMode {

    private Follower follower;
    private DcMotorEx launchMotor;
    private DcMotorEx turretMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx intakeMotor;
    private Servo gateServo;
    private CRServo contServo;

    private boolean shooting = false;
    private double gatePos = 0.5;
    private double intakeTPS = 0;
    private double shooterTPS = 720;
    private double curTransferTps;
    private double turretPower;
    private double baseTransferTps = 575;


    private boolean lastRightTrigger2 = false;
    private boolean lastLeftTrigger2 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastDpadRight2 = false;
    private boolean lastDpadLeft2 = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastY2 = false;
    private boolean lastX2 = false;

    @Override
    public void runOpMode() {

        follower = Constants.create(hardwareMap);

        // Initialized all hardware components
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        contServo = hardwareMap.get(CRServo.class, "contServo");

        // Set reverse directions if necessary

        curTransferTps = baseTransferTps;
        contServo.setPower(0);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            follower.update();

            // Drive inputs
            double axial = -gamepad1.right_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            follower.manual(axial, lateral, turn);

            intakeLogic();
            tpsAdjustments();
            shooterLogic();
            settingPowers();
            postData();

            // Update button states at the end of the loop iteration
            updateButtonStates();
        }
    }

    private void updateButtonStates() {
        lastRightTrigger2 = gamepad2.right_trigger > 0.5;
        lastLeftTrigger2 = gamepad2.left_trigger > 0.5;
        lastLeftBumper2 = gamepad2.left_bumper;
        lastDpadRight2 = gamepad2.dpad_right;
        lastDpadLeft2 = gamepad2.dpad_left;
        lastDpadUp2 = gamepad2.dpad_up;
        lastDpadDown2 = gamepad2.dpad_down;
        lastY2 = gamepad2.y;
        lastX2 = gamepad2.x;
    }

    private void intakeLogic() {
        // Detect single presses using rising-edge logic
        boolean rightTriggerPressed = (gamepad2.right_trigger > 0.5) && !lastRightTrigger2;
        boolean leftTriggerPressed = (gamepad2.left_trigger > 0.5) && !lastLeftTrigger2;
        boolean leftBumperPressed = gamepad2.left_bumper && !lastLeftBumper2;

        if (rightTriggerPressed && !shooting) {
            transferMotor.setVelocity(curTransferTps);
            intakeTPS = 1150;
        }
        if (leftTriggerPressed && !shooting) {
            transferMotor.setVelocity(225);
            intakeTPS = 0;
        }
        if (leftBumperPressed && !shooting) {
            transferMotor.setVelocity(-curTransferTps);
            intakeTPS = -1150;
        }
    }

    private void tpsAdjustments() {
        boolean dpadRightPressed = gamepad2.dpad_right && !lastDpadRight2;
        boolean dpadLeftPressed = gamepad2.dpad_left && !lastDpadLeft2;
        boolean dpadUpPressed = gamepad2.dpad_up && !lastDpadUp2;
        boolean dpadDownPressed = gamepad2.dpad_down && !lastDpadDown2;

        if (dpadRightPressed && !shooting) curTransferTps += 15;
        if (dpadLeftPressed && !shooting) curTransferTps -= 15;
        if (dpadUpPressed) shooterTPS += 15;
        if (dpadDownPressed) shooterTPS -= 15;
    }

    private void shooterLogic() {
        boolean yPressed = gamepad2.y && !lastY2;
        boolean xPressed = gamepad2.x && !lastX2;

        if (yPressed) {
            shooting = true;
            transferMotor.setVelocity(1150);
            intakeTPS = 1150;
            gatePos = 0.5;
            contServo.setPower(1.0);
        }
        if (xPressed) {
            shooting = false;
            transferMotor.setVelocity(curTransferTps);
            intakeTPS = 1150;
            gatePos = 0.0;
            contServo.setPower(0);
        }
        turretPower = gamepad2.right_stick_x;
    }

    private void settingPowers() {
        gateServo.setPosition(gatePos);
        intakeMotor.setVelocity(intakeTPS);
        launchMotor.setVelocity(shooterTPS);
        turretMotor.setPower(turretPower);
    }
    private void postData(){
        telemetry.addData("Target ShooterTPS: ", shooterTPS);
        telemetry.addData("Current ShooterTPS: ", launchMotor.getVelocity());
        telemetry.addData("Is Shooting: ", shooting);
        telemetry.addData("Gate Open ", gatePos == 0.0);
        telemetry.update();
    }
}