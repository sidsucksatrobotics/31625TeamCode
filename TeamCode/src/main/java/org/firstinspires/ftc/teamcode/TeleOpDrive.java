// Don't delete lines 1 through 10, it is importing packages and libraries, and declaring the class
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp Drive with Toggle Precision Mode", group="Linear Opmode")
public class TeleOpDrive extends LinearOpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear, mainShooter;
    private CRServo leftShooter, rightShooter;

    private boolean precisionMode = false; // starts full speed
    private boolean lastBumperState = false; // for detecting button press of slow mode
    private boolean shooterMode = false; // starts with shooters off
    private boolean lastShooterState = false; // for detecting button press of shooter
    private long shooterStartTime = 0;

    @Override
    public void runOpMode() {
        // Map motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        mainShooter = hardwareMap.get(DcMotor.class, "mainShooter");
        leftShooter = hardwareMap.get(CRServo.class, "leftShooter");
        rightShooter = hardwareMap.get(CRServo.class, "rightShooter");

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run using the encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mainShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse some motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftShooter.setDirection(CRServo.Direction.FORWARD);
        rightShooter.setDirection(CRServo.Direction.FORWARD); // or REVERSE if it spins opposite

        // Report initialized status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait to start
        waitForStart();

        while (opModeIsActive()) {

            // Joystick inputs
            double y  = -gamepad1.left_stick_y;  // forward/back
            double x  =  gamepad1.left_stick_x;  // strafe
            double rx =  gamepad1.right_stick_x; // rotation

            // Calculate motor powers
            double frontLeftPower  = y + x + rx;
            double backLeftPower   = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower  = y + x - rx;

            // Normalize
            double max = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(backLeftPower),
                                    Math.max(Math.abs(frontRightPower),
                                            Math.abs(backRightPower)))));

            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;

            // Toggle precision mode on left bumper press
            boolean currentBumper = gamepad1.left_bumper;
            if (currentBumper && !lastBumperState) {
                precisionMode = !precisionMode;
            }
            lastBumperState = currentBumper;

            // Apply precision scaling
            if (precisionMode) {
                frontLeftPower  *= 0.5;
                backLeftPower   *= 0.5;
                frontRightPower *= 0.5;
                backRightPower  *= 0.5;
            }


            // Toggle shooter mode when A is pressed
            boolean currentShooterButton = gamepad2.a;
            if (currentShooterButton && !lastShooterState) {
                shooterMode = !shooterMode;

                if (shooterMode) {
                    // Start main shooter motor and record time
                    mainShooter.setPower(1.0);
                    shooterStartTime = System.currentTimeMillis(); // mark start
                } else {
                    // Stop everything
                    mainShooter.setPower(0);
                    leftShooter.setPower(0);
                    rightShooter.setPower(0);
                }
            }
            lastShooterState = currentShooterButton;

            // After toggling, check if 2 seconds have passed since main shooter started
            if (shooterMode && System.currentTimeMillis() - shooterStartTime >= 2000) {
                // Feed balls after spin-up delay
                leftShooter.setPower(-1.0);
                rightShooter.setPower(1.0);
            }



            // Set motor powers
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // Add the data for motors and precision mode and shooter mode
            telemetry.addData("Drive Mode", precisionMode ? "Precision (Half Speed)" : "Full Speed");
            telemetry.addData("Shooter Mode", shooterMode ? "On" : "Off");
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Right", backRightPower);

            // Add the data for encoders
            telemetry.addData("Front Left Encoder", leftFront.getCurrentPosition());
            telemetry.addData("Front Right Encoder", rightFront.getCurrentPosition());
            telemetry.addData("Back Left Encoder", leftRear.getCurrentPosition());
            telemetry.addData("Back Right Encoder", rightRear.getCurrentPosition());
            telemetry.addData("Shooter Encoder", mainShooter.getCurrentPosition());

            // Updating data
            telemetry.update();
        }
    }
}