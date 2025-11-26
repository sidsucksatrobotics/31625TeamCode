package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="IMU Test", group="Test")
public class IMUTest extends LinearOpMode {

    private BNO055IMU imu;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing IMU...");
        telemetry.update();

        // Map the IMU (make sure the name matches the config)
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.useExternalCrystal = true; // sometimes helps

        imu.initialize(parameters);

        // Wait for calibration
        telemetry.addLine("Waiting for IMU calibration...");
        telemetry.update();
        while (!imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addLine("IMU Calibrating...");
            telemetry.update();
        }

        telemetry.addLine("IMU Calibrated!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;

            telemetry.addData("Heading (Z)", heading);
            telemetry.update();
        }
    }
}
