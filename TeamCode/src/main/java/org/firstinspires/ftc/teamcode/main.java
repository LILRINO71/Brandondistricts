package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class main extends LinearOpMode {
    DcMotor arm, fR, fL, bL, bR, wrist;
    Servo leftClaw, rightClaw, leftClip, rightClip;
    BNO055IMU imu;
    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        bR = hardwareMap.dcMotor.get("bR");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        fL = hardwareMap.dcMotor.get("fL");
        wrist = hardwareMap.dcMotor.get("wrist");
        arm = hardwareMap.dcMotor.get("arm");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClip");
        leftClip = hardwareMap.servo.get("leftClip");
        rightClip = hardwareMap.servo.get("rightClip");


        fR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

            double drivePower = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepadRateLimit.hasExpired() && gamepad1.options) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            double power = ((adjustedLy - adjustedLx + rx) / max) * drivePower;

            bR.setPower(power);
            bL.setPower(power);
            fR.setPower(power);
            fL.setPower(power);
            //what the sigma - Joel
//
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            bL.setPower(backLeftPower);
//            fL.setPower(frontLeftPower);
//            bR.setPower(backRightPower);
//            fR.setPower(frontRightPower);

            // arm
            if (Math.abs(gamepad2.right_stick_x) > .2) {
                arm.setPower(gamepad2.right_stick_y *1);
                while (gamepad2.right_bumper) {
                    arm.setPower(gamepad2.right_stick_x *.50);

                    bR.setPower(power);
                    bL.setPower(power);
                    fR.setPower(power);
                    fL.setPower(power);
                }
            } else {
                arm.setPower(0);
            }

            //wrist
            if (Math.abs(gamepad2.left_stick_y) > .2) {
                wrist.setPower(gamepad2.left_stick_y * 1);
                while (gamepad2.right_bumper) {
                    wrist.setPower(gamepad2.left_stick_y *.50);

                    bR.setPower(power);
                    bL.setPower(power);
                    fR.setPower(power);
                    fL.setPower(power);
                }
            } else {
                wrist.setPower(0);
            }

            //claw
            if (gamepad2.a) {
                leftClaw.setPosition(0.5);//close
                rightClaw.setPosition(0.5);
            } else {
                leftClaw.setPosition(0);//open
                rightClaw.setPosition(0);
            }

            //clip
            if (gamepad2.x) {
                leftClip.setPosition(0.5);//up
                rightClip.setPosition(0.5);
            } else {
                leftClip.setPosition(0);//down
                rightClip.setPosition(0);
            }
        }
    }
}