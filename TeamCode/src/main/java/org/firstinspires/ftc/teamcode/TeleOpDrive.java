// Don't delete lines 1 through 10, it is importing packages and libraries, and declaring the class
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TeleOp Drive with Toggle Precision Mode", group="Linear Opmode")
public class TeleOpDrive extends LinearOpMode {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private CRServo leftShooter, rightShooter;
    private DcMotorEx mainShooter;

    private boolean precisionMode = false; // starts full speed
    private boolean lastBumperState = false; // for detecting button press of slow mode
    private boolean shooterMode = false; // starts with shooters off

    // Feeder timing variables
    private boolean lastShooterButton = false;

    private boolean feedTriggered = false;      // B button: single feed trigger
    private boolean lastFeedButton = false;
    private long feedStartTime = 0;

    private final long FEED_MS = 300;          // feeders run duration



    @Override
    public void runOpMode() {
        // Map motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        leftShooter = hardwareMap.get(CRServo.class, "leftShooter");
        rightShooter = hardwareMap.get(CRServo.class, "rightShooter");

        // --- Shooter uses velocity control ---
        mainShooter = hardwareMap.get(DcMotorEx.class, "mainShooter");

        // Reset encoder for shooter
        mainShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // Deadzone filter
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;


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



            // --- Toggle shooter with A ---
            boolean currentShooterButton = gamepad2.a;
            if (currentShooterButton && !lastShooterButton) {
                shooterMode = !shooterMode;

                if (shooterMode) {
                    mainShooter.setVelocity(2000);
                } else {
                    mainShooter.setVelocity(0);
                }

            }
            lastShooterButton = currentShooterButton;

            // --- Trigger feed with B ---
            boolean currentFeedButton = gamepad2.b;
            if (currentFeedButton && !lastFeedButton) {
                if (shooterMode) {
                    // Only feed if shooter is spinning
                    leftShooter.setPower(-1.0);
                    rightShooter.setPower(1.0);
                    feedStartTime = System.currentTimeMillis();
                    feedTriggered = true;
                }
            }
            lastFeedButton = currentFeedButton;

            // --- Stop feeders after FEED_MS ---
            if (feedTriggered && System.currentTimeMillis() - feedStartTime >= FEED_MS) {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
                feedTriggered = false;  // reset trigger
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
            telemetry.addData("Shooter Velocity (tps)", mainShooter.getVelocity());


            // Updating data
            telemetry.update();
        }
    }
}