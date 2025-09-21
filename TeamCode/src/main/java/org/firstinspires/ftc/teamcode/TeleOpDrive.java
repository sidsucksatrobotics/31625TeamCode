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

    @Override
    public void runOpMode() {
        // Map motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        leftShooter = hardwareMap.get(CRServo.class, "leftShooter");
        rightShooter = hardwareMap.get(CRServo.class, "rightShooter");

        // Reverse right side motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        // Report initialized status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait to start
        waitForStart();

        while (opModeIsActive()) {
            // Toggle precision mode on bumper press
            if (gamepad1.left_bumper && !lastBumperState) {
                precisionMode = !precisionMode;
            }
            lastBumperState = gamepad1.left_bumper;

            // Toggle shooter mode on button press of "A"
            if (gamepad1.a && !lastShooterState) {
                shooterMode=!shooterMode;
            }
            lastShooterState = gamepad1.a;

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

            // Apply precision mode if toggled
            if (precisionMode) {
                frontLeftPower  *= 0.5;
                backLeftPower   *= 0.5;
                frontRightPower *= 0.5;
                backRightPower  *= 0.5;
            }

            if (shooterMode) {
                mainShooter.setPower(-1);
                leftShooter.setPower(-1);
                rightShooter.setPower(1);
            }

            // Set motor powers
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            telemetry.addData("Mode", precisionMode ? "Precision (Half Speed)" : "Full Speed");
            telemetry.addData("Mode", shooterMode ? "Shooter" + "On" : "Off");
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
