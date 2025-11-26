package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpCurry")
public class TeleOpCurry extends LinearOpMode {

    // ----------------- DRIVE MOTORS -----------------
    DcMotorEx leftFront, rightFront, leftBack, rightBack;
    DcMotorEx leftOdo, rightOdo, centerOdo;

    // ----------------- IMU -----------------
    private IMU imu;

    // ----------------- ODOMETRY CONSTANTS -----------------
    final double TICKS_PER_REV = 8192;
    final double WHEEL_DIAMETER_IN = 2.0;
    final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    final double TRACK_WIDTH = 13.5;
    final double CENTER_WHEEL_OFFSET = -6.0;

    // ----------------- POSE -----------------
    double x = 0, y = 0, heading = 0;
    int lastL = 0, lastR = 0, lastC = 0;

    // ----------------- HEADING LOCK -----------------
    boolean headingLock = false;   // whether heading lock is active
    double lockedHeading = 0;

    // ----------------- PRECISION MODE -----------------
    private boolean precisionMode = false; // starts full speed
    private boolean lastBumperState = false; // for detecting button press of slow mode


    @Override
    public void runOpMode() {

        // ---------- HARDWARE MAP ----------
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftOdo   = leftBack;
        rightOdo  = rightBack;
        centerOdo = leftFront;

        // Reverse motors as needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize odometry encoders
        lastL = leftOdo.getCurrentPosition();
        lastR = rightOdo.getCurrentPosition();
        lastC = centerOdo.getCurrentPosition();

        // ---------- IMU INITIALIZATION ----------
        imu = hardwareMap.get(IMU.class, "imu");

        imu.resetYaw();


        telemetry.addLine("Ready!");
        telemetry.update();
        waitForStart();

        // --------------- TELEOP LOOP -----------------
        while (opModeIsActive()) {

            updateOdometry();

            if (gamepad1.a) {
                x = 0;
                y = 0;
            }

            if (gamepad1.b) {
                imu.resetYaw();
            }

            // Activate heading lock when right trigger is pressed
            if (gamepad1.right_trigger > 0.5 && !headingLock) {
                headingLock = true;
                lockedHeading = heading;  // store current heading
            }

            // Optionally: release lock when trigger released
            if (gamepad1.right_trigger < 0.5) {
                headingLock = false;
            }

            // Use lockedHeading instead of live heading if headingLock is true
            double driveHeading = headingLock ? lockedHeading : heading;



            // --- FIELD-CENTRIC DRIVE ---
            double yInput = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
            double xInput = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
            double turnInput = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;


            // Rotate joystick inputs by -heading for field-centric
            double temp = yInput * Math.cos(-driveHeading) - xInput * Math.sin(-driveHeading);
            xInput = yInput * Math.sin(-driveHeading) + xInput * Math.cos(-driveHeading);
            yInput = temp;

            double lf = yInput + xInput + turnInput;
            double rf = yInput - xInput - turnInput;
            double lb = yInput - xInput + turnInput;
            double rb = yInput + xInput - turnInput;

            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lb), Math.abs(rb)));

            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }


            // Toggle precision mode on left bumper press
            boolean currentBumper = gamepad1.left_bumper;
            if (currentBumper && !lastBumperState) {
                precisionMode = !precisionMode;
            }
            lastBumperState = currentBumper;

            if (precisionMode) {
                lf  *= 0.5;
                lb   *= 0.5;
                rf *= 0.5;
                rb  *= 0.5;
            }


            leftFront.setPower(lf);
            rightFront.setPower(rf);
            leftBack.setPower(lb);
            rightBack.setPower(rb);

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.update();
        }
    }

    // ----------------- ODOMETRY UPDATE -----------------
    public void updateOdometry() {

        int newL = leftOdo.getCurrentPosition();
        int newR = rightOdo.getCurrentPosition();
        int newC = centerOdo.getCurrentPosition();

        double dL = (newL - lastL)/TICKS_PER_INCH;
        double dR = (newR - lastR)/TICKS_PER_INCH;
        double dC = (newC - lastC)/TICKS_PER_INCH;

        if (Math.abs(dL) < 0.01) dL = 0;
        if (Math.abs(dR) < 0.01) dR = 0;
        if (Math.abs(dC) < 0.01) dC = 0;


        lastL = newL; lastR = newR; lastC = newC;

        // Use Hub IMU for heading
        heading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw());


        // Encoder-based delta heading (small correction)
        double dThetaEnc = (dR - dL)/TRACK_WIDTH;

        // Robot-frame movement
        double dx, dy;
        if(Math.abs(dThetaEnc)<1e-6){
            dx = dC;
            dy = (dL+dR)/2.0;
        } else{
            double r = (dL+dR)/(2*dThetaEnc);
            double strafeRadius = dC/dThetaEnc;
            dy = r*Math.sin(dThetaEnc);
            dx = strafeRadius*Math.sin(dThetaEnc);
        }

        // Convert to global coordinates
        double globalDX = dx*Math.cos(heading) - dy*Math.sin(heading);
        double globalDY = dx*Math.sin(heading) + dy*Math.cos(heading);

        x += globalDX;
        y += globalDY;
    }
}
