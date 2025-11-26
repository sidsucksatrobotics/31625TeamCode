package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import java.util.ArrayList;




@Autonomous(name="AutonomousCurry")
public class AutonomousCurry extends LinearOpMode {

    DcMotorEx leftFront, rightFront, leftBack, rightBack;
    DcMotorEx leftOdo, rightOdo, centerOdo;
    private IMU imu;

    final double TICKS_PER_REV = 8192;
    final double WHEEL_DIAMETER_IN = 2.0;
    final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);
    final double TRACK_WIDTH = 13.5;
    final double CENTER_WHEEL_OFFSET = -6.0;

    double x=0, y=0, heading=0;
    int lastL=0, lastR=0, lastC=0;

    double lastHeadingError = 0;

    private ArrayList<AprilTagDetection> latestDetections = new ArrayList<>();

    private static final double fx = 578.272; // Focal lengths
    private static final double fy = 578.272;
    private static final double cx = 402.145; // Optical center
    private static final double cy = 221.506;

    private static final double TAG_SIZE = 0.166;

    private long nativeDetectorPtr;
    private Mat gray = new Mat();

    int alliance = -1;   // 1 = BLUE, 2 = RED
    int motif = -1;      // motif decision


    public void init(Mat firstFrame) {
        nativeDetectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    class AprilTagPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

            latestDetections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                    nativeDetectorPtr, gray, TAG_SIZE, fx, fy, cx, cy
            );

            for (AprilTagDetection detection : latestDetections) {

                switch (detection.id) {

                    case 20:
                        alliance = 1;
                        telemetry.addLine("Alliance: BLUE");
                        break;

                    case 24:
                        alliance = 2;
                        telemetry.addLine("Alliance: RED");
                        break;

                    case 21:
                        motif = 1;
                        telemetry.addLine("Motif Pattern GPP");
                        break;

                    case 22:
                        motif = 2;
                        telemetry.addLine("Motif Pattern PGP");
                        break;

                    case 23:
                        motif = 3;
                        telemetry.addLine("Motif Pattern PPG");
                        break;
                }
            }
            return input;
        }
    }

    @Override
        public void runOpMode(){

            leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            leftBack   = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack  = hardwareMap.get(DcMotorEx.class, "rightBack");

            leftOdo = leftBack;
            rightOdo = rightBack;
            centerOdo = leftFront;

            rightFront.setDirection(DcMotorEx.Direction.REVERSE);
            rightBack.setDirection(DcMotorEx.Direction.REVERSE);

            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            lastL = leftOdo.getCurrentPosition();
            lastR = rightOdo.getCurrentPosition();
            lastC = centerOdo.getCurrentPosition();

            imu = hardwareMap.get(IMU.class,"imu");

            imu.resetYaw();
            heading = 0;

            telemetry.addLine("Ready!");
            telemetry.update();

            int cameraMonitorViewId = hardwareMap.appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, "Webcam 1"),
                    cameraMonitorViewId
            );

            webcam.setPipeline(new AprilTagPipeline());

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(800, 448);
                }

                @Override
                public void onError(int errorCode) { }
            });


            waitForStart();
            // --------------- AUTO STARTS HERE ----------------

            /*Here is what the auto should do
            1. Starting with the shooter facing the goal, move back and detect what alliance the robot is on and storing in the variable alliance. Either before or after that, we should shoot the 3 preloaded balls
            2. Go and detect the motif pattern on the obelisk
            3. Determine which set of balls to collect and shoot them into the goal


            */
        }


        //-------------------- The things after this are defining methods. don't touch them unless they need to be changed -----------------

    // ---------------- ODOMETRY UPDATE ----------------
        public void updateOdometry(){
        int newL = leftOdo.getCurrentPosition();
        int newR = rightOdo.getCurrentPosition();
        int newC = centerOdo.getCurrentPosition();

        double dL=(newL-lastL)/TICKS_PER_INCH;
        double dR=(newR-lastR)/TICKS_PER_INCH;
        double dC=(newC-lastC)/TICKS_PER_INCH;

        lastL=newL; lastR=newR; lastC=newC;

        heading = imu.getRobotYawPitchRollAngles().getYaw(); // radians

        double dThetaEnc=(dR-dL)/TRACK_WIDTH;

        double dx,dy;
        if(Math.abs(dThetaEnc)<1e-6){
            dx=dC; dy=(dL+dR)/2.0;
        } else{
            double r=(dL+dR)/(2*dThetaEnc);
            double strafeRadius=dC/dThetaEnc;
            dy=r*Math.sin(dThetaEnc);
            dx=strafeRadius*Math.sin(dThetaEnc);
        }

        double globalDX=dx*Math.cos(heading)-dy*Math.sin(heading);
        double globalDY=dx*Math.sin(heading)+dy*Math.cos(heading);

        x+=globalDX; y+=globalDY;
    }


    // Wrap angle to [-PI, PI]
    public double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    // ---------------- DRIVE TO POSITION (with heading hold) ----------------


    public void driveToPosition(double targetX, double targetY, double targetHeadingRad, double maxPower){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double timeout = 5.0;

        while(opModeIsActive() && Math.hypot(targetX-x, targetY-y) > 0.5 && timer.seconds() < timeout){
            updateOdometry();

            double dx = targetX - x;
            double dy = targetY - y;
            double distance = Math.hypot(dx, dy);
            double movePower = Math.min(maxPower, distance * 0.4); // kP for distance

            double moveAngle = Math.atan2(dy, dx) - heading;

            double headingError = wrapAngle(targetHeadingRad - heading);
            double derivative = headingError - lastHeadingError;
            lastHeadingError = headingError;
            double turnCorrection = headingError *  0.6+ derivative * 0.05;

            double vx = Math.cos(moveAngle) * movePower;
            double vy = Math.sin(moveAngle) * movePower;

            leftFront.setPower(vy + vx + turnCorrection);
            rightFront.setPower(vy - vx - turnCorrection);
            leftBack.setPower(vy - vx + turnCorrection);
            rightBack.setPower(vy + vx - turnCorrection);

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
        stopAllMotors();
    }


    // ---------------- TURN TO HEADING (PID upgraded) ----------------
    public void turnToHeading(double targetHeadingDeg, double maxPower){
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);
        double kP = 1.0;
        double kD = 0.1;

        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double timeout = 5.0;

        while(opModeIsActive() && timer.seconds() < timeout){
            updateOdometry();
            double error = wrapAngle(targetHeadingRad - heading);
            if(Math.abs(error) < Math.toRadians(1)) break;

            double derivative = error - lastError;
            lastError = error;

            double output = kP * error + kD * derivative;
            output = Math.max(-maxPower, Math.min(maxPower, output));

            leftFront.setPower(output);
            leftBack.setPower(output);
            rightFront.setPower(-output);
            rightBack.setPower(-output);

            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.addData("Error", Math.toDegrees(error));
            telemetry.update();
        }
        stopAllMotors();
    }


    // ---------------- DRIVE FORWARD USING ODOMETRY ----------------
    public void driveForward(double inches, double power){
        updateOdometry();

        double targetX = x + inches * Math.cos(heading);
        double targetY = y + inches * Math.sin(heading);

        driveToPosition(targetX, targetY, heading, power);
    }

    // ---------------- STRAFE USING ODOMETRY ----------------
    public void driveStrafe(double inches, double power){
        updateOdometry();

        double targetX = x + inches * -Math.sin(heading);
        double targetY = y + inches * Math.cos(heading);

        driveToPosition(targetX, targetY, heading, power);
    }

    // ---------------- GOTO POSE: Drive + Turn Same Time ----------------
    public void goToPose(double targetX, double targetY, double targetHeadingDeg, double power){
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        while(opModeIsActive()){

            updateOdometry();

            double dx = targetX - x;
            double dy = targetY - y;
            double dist = Math.hypot(dx, dy);

            if(dist < 1 && Math.abs(targetHeadingRad - heading) < Math.toRadians(1)) break;

            double movementAngle = Math.atan2(dy, dx) - heading;

            double vx = Math.cos(movementAngle) * power;
            double vy = Math.sin(movementAngle) * power;

            double headingError = targetHeadingRad - heading;
            double turn = headingError * 0.7;

            leftFront.setPower(vy + vx + turn);
            rightFront.setPower(vy - vx - turn);
            leftBack.setPower(vy - vx + turn);
            rightBack.setPower(vy + vx - turn);

            telemetry.addData("GoTo X", x);
            telemetry.addData("GoTo Y", y);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }
        stopAllMotors();
    }

    // ---------------- STOP MOTORS ----------------
    public void stopAllMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
