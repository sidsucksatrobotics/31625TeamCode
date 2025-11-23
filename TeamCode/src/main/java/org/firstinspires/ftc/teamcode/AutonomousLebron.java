package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="AutonomousLebron", group="Robot")
public class AutonomousLebron extends LinearOpMode {



    // Drive motors
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    // Shooter motors/servos
    private DcMotorEx mainShooter;
    private CRServo leftShooter, rightShooter;

    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;

    // Constants for encoder calculations
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 312 RPM encoder counts per revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0;      // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // Diameter of GoBilda mecanum wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI); // Counts per inch of wheel travel


    static final double TRACK_WIDTH = 14.0; // Distance between left and right wheels (center-to-center) in inches



    @Override
    public void runOpMode() {

        // IMU setup
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));



        // Map hardware names from Robot Configuration to variables
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        mainShooter = hardwareMap.get(DcMotorEx.class, "mainShooter");
        leftShooter = hardwareMap.get(CRServo.class, "leftShooter");
        rightShooter = hardwareMap.get(CRServo.class, "rightShooter");

        // Reverse motors if necessary so positive power moves robot forward
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders to start from zero
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run motors using encoders (so we can track movement)
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Display starting encoder positions for all four drive motors
        telemetry.addData("Starting Pos",
                "FL:%7d FR:%7d BL:%7d BR:%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();

        // START CODE HERE

        driveStraightWithIMU(1, 24, 1);

        shooterMode(3);
        sleep(1500);

        driveStraightWithIMU(1, 96, 1);

        // END CODE HERE
    }

    /**
     * Converts degrees to linear inches each wheel must travel
     * @param degrees Turn angle (+ right, - left)
     * @return Linear inches per wheel
     */
    public double degreesToInches(double degrees) {
        return Math.PI * TRACK_WIDTH * (degrees / 360.0);
    }

    // Calculate robot's current angle
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public void shooterMode(int cycles) {
        final int SPINUP_MS = 7000;  // time for shooter to reach speed
        final int FEED_MS = 530;     // how long feeders run

        for (int i=0; i < cycles; i++){
            // Step 1: start main shooter
            mainShooter.setPower(0.5);
            telemetry.addLine("Shooter spinning up...");
            telemetry.update();
            sleep(SPINUP_MS); // wait for spin-up

            // Step 2: start feeders
            leftShooter.setPower(-1);
            rightShooter.setPower(1);
            telemetry.addLine("Feeding...");
            telemetry.update();
            sleep(FEED_MS); // run feeders

            // Step 3: stop feeders and shooter
            leftShooter.setPower(0);
            rightShooter.setPower(0);

            telemetry.addLine("Shooter cycle complete");
            telemetry.update();

        }
        mainShooter.setVelocity(0);

    }
    public void imuTurn(double targetAngle, double power) {
        // targetAngle in degrees, positive = right, negative = left
        double error = targetAngle - getHeading();

        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        while (Math.abs(error) > 1 && opModeIsActive()) {
            double turnPower = power * Math.signum(error);
            // Turn
            leftFront.setPower(-turnPower);
            leftRear.setPower(turnPower);
            rightFront.setPower(-turnPower);
            rightRear.setPower(-turnPower);

            error = targetAngle - getHeading();
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }

        // Stop all motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }




    public void driveStraightWithIMU(double speed, double inches, double timeoutS) {


        int targetCounts = (int)(Math.abs(inches) * COUNTS_PER_INCH);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double startAngle = getHeading(); // safe now
        runtime.reset();
        double dir = inches >= 0 ? 1 : -1;
        double basePower = speed * dir;

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            // Average absolute encoder counts
            int avgPos = (Math.abs(leftFront.getCurrentPosition()) +
                    Math.abs(rightFront.getCurrentPosition()) +
                    Math.abs(leftRear.getCurrentPosition()) +
                    Math.abs(rightRear.getCurrentPosition())) / 4;

            if (avgPos >= targetCounts) break;

            // Calculate heading error
            double error = getHeading() - startAngle;
            // Normalize to [-180, 180]
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double correction = error * 0.045;  // tune P gain

            leftFront.setPower(basePower - correction);
            leftRear.setPower(basePower - correction);
            rightFront.setPower(basePower + correction);
            rightRear.setPower(basePower + correction);

            // Telemetry for debugging
            telemetry.addData("Target Counts", targetCounts);
            telemetry.addData("Avg Pos", avgPos);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Stop all motors
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }


}
