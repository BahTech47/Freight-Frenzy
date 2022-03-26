package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="TeleOp", group ="Concept")
public class FFTele extends LinearOpMode {

    // Declaration for the hardware variables
    final FFMovement movement = new FFMovement(telemetry);

    @Override
    public void runOpMode() {
        movement.initHardware(hardwareMap);

        // Program to run once when we press start
        waitForStart();

        // Program to run continuously after ew press start
        while (opModeIsActive()) {

            movement.move(gamepad1.left_stick_x, gamepad1.left_stick_y);
//            movement.carousel(gamepad1.b);
//            movement.intake(gamepad1.a);
//            movement.trigger(gamepad1.left_bumper);
//            movement.arm(gamepad1.right_stick_x);
        }
    }
}