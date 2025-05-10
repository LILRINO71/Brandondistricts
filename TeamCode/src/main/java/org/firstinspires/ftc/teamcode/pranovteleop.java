package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="LIL_RINO_Full_Strafe_Move", group="Robot")
public class pranovteleop extends LinearOpMode {

    DcMotor bL, fL, bR, fR;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        bL = hardwareMap.get(DcMotor.class, "bL");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fR = hardwareMap.get(DcMotor.class, "fR");


        // Reverse left motors so all move forward with same power
        bL.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

//FIELD CENTRIC
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
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

        while (opModeIsActive()) {
            // Get forward/back from left stick Y
            double drive = -gamepad1.left_stick_y;

            // Get strafe from right stick X
            double strafe = gamepad1.right_stick_x;

            // Combine for mecanum power
            double frontLeftPower  = drive + strafe;
            double frontRightPower = drive - strafe;
            double backLeftPower   = drive - strafe;
            double backRightPower  = drive + strafe;

            // Normalize so no value goes over 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Set powers
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);





            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.update();
        }
    }
}
