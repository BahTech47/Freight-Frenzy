package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "FFTest", group = "BahTech")
public class FFTest extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");

        waitForStart();

    }
}
