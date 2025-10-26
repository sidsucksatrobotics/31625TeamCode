package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Mecanum: Auto Template", group="Robot")
public class AutonomousTemplate extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    // Shooter motors/servos
    private DcMotor mainShooter;
    private CRServo leftShooter, rightShooter;

    private ElapsedTime runtime = new ElapsedTime();

    private BNO055IMU imu;

    // Constants for encoder calculations
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 312 RPM encoder counts per revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0;      // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // Diameter of GoBilda mecanum wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI); // Counts per inch of wheel travel

    static final double DRIVE_SPEED = 0.6;  // Speed for forward/back/strafe
    static final double TURN_SPEED  = 0.5;  // Speed for turning
    static final double TRACK_WIDTH = 14.0; // Distance between left and right wheels (center-to-center) in inches

    @Override
    public void runOpMode() {

        // Map hardware names from Robot Configuration to variables
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");
        mainShooter = hardwareMap.get(DcMotor.class, "mainShooter");
        leftShooter = hardwareMap.get(CRServo.class, "leftShooter");
        rightShooter = hardwareMap.get(CRServo.class, "rightShooter");

        // Reverse motors if necessary so positive power moves robot forward
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Reverse if needed

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

        // IMU setup
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Wait for IMU calibration
        while (!imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addData("IMU", "Calibrating...");
            telemetry.update();
        }
        telemetry.addData("IMU", "Calibrated!");
        telemetry.update();


        // Wait for the driver to press START
        waitForStart();

        // START CODE HERE

        // Example code

        // Move forward 48 inches
        encoderDriveMecanum(DRIVE_SPEED, 48, 0, 0, 5.0);

        // Strafe right 12 inches
        encoderDriveMecanum(DRIVE_SPEED, 0, 12, 0, 4.0);

        // Turn right 90 degrees
        imuTurn(90, 0.3);

        // Turn on shooter for 2000 ms
        shooterMode(2000);

        // Move backward 24 inches
        encoderDriveMecanum(DRIVE_SPEED, -24, 0, 0, 4.0);

        // End of autonomous path
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

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
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; // Z-axis rotation
    }


    public void shooterMode(int i) {
        final int SPINUP_MS = 3000;  // time for shooter to reach speed
        final int FEED_MS = 300;     // how long feeders run

        // Step 1: start main shooter
        mainShooter.setPower(1);
        telemetry.addLine("Shooter spinning up...");
        telemetry.update();
        sleep(SPINUP_MS); // wait for spin-up

        // Step 2: start feeders
        leftShooter.setPower(-1.0);
        rightShooter.setPower(1.0);
        telemetry.addLine("Feeding...");
        telemetry.update();
        sleep(FEED_MS); // run feeders

        // Step 3: stop feeders and shooter
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        mainShooter.setPower(0);
        telemetry.addLine("Shooter cycle complete");
        telemetry.update();
    }
    public void imuTurn(double targetAngle, double power) {
        // targetAngle in degrees, positive = right, negative = left
        double error = targetAngle - getHeading();

        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        while (Math.abs(error) > 1 && opModeIsActive()) {
            double turnPower = power * Math.signum(error);
            // Turns
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


    /**
     * Moves the mecanum robot based on forward/backward, strafe, and turn (in degrees).
     *
     * @param speed   Motor power (0-1)
     * @param forward Inches to move forward/back (+forward, -back)
     * @param strafe  Inches to move right/left (+right, -left)
     * @param turnDeg Degrees to turn (+right, -left)
     * @param timeoutS Maximum time to complete the move
     */


    public void encoderDriveMecanum(double speed, double forward, double strafe, double turnDeg, double timeoutS) {

        if (opModeIsActive()) {

            // Convert turn degrees to inches for wheel travel
            double turnInches = Math.PI * TRACK_WIDTH * (turnDeg / 360.0);

            // Calculate target positions for all four wheels
            int flTarget = leftFront.getCurrentPosition() + (int)((forward + strafe + turnInches) * COUNTS_PER_INCH);
            int frTarget = rightFront.getCurrentPosition() + (int)((forward - strafe - turnInches) * COUNTS_PER_INCH);
            int blTarget = leftRear.getCurrentPosition() + (int)((forward - strafe + turnInches) * COUNTS_PER_INCH);
            int brTarget = rightRear.getCurrentPosition() + (int)((forward + strafe - turnInches) * COUNTS_PER_INCH);


    // Set target positions for RUN_TO_POSITION mode
            leftFront.setTargetPosition(flTarget);
            rightFront.setTargetPosition(frTarget);
            leftRear.setTargetPosition(blTarget);
            rightRear.setTargetPosition(brTarget);

            // Enable RUN_TO_POSITION mode
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // Keep looping while motors are moving and within timeout
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    leftFront.isBusy() && rightFront.isBusy() &&
                    leftRear.isBusy() && rightRear.isBusy()) {

                // Show target vs current position for debugging
                telemetry.addData("FL Target/Current", "%7d / %7d", flTarget, leftFront.getCurrentPosition());
                telemetry.addData("FR Target/Current", "%7d / %7d", frTarget, rightFront.getCurrentPosition());
                telemetry.addData("BL Target/Current", "%7d / %7d", blTarget, leftRear.getCurrentPosition());
                telemetry.addData("BR Target/Current", "%7d / %7d", brTarget, rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motors
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Return motors to RUN_USING_ENCODER mode
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Small pause after each move
            sleep(250);
        }
    }
}
