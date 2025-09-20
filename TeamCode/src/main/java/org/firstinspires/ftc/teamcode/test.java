package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="test")
public class test extends OpMode{
    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "test motor");
        telemetry.addData("Hardware: ", "Initialized");


    }

    @Override
    public void loop() {

        if (gamepad1.a){
            motor.setPower(1);
        }
        motor.setPower(0);

    }
}