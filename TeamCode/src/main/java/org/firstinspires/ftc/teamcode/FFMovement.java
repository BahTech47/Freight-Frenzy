package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FFMovement {

    final Telemetry telemetry;
    final double smoother = 0.015;

    DcMotor arm;
    DcMotor intake;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor carousel;
    Servo ejector;

    public FFMovement(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void initHardware(@NonNull HardwareMap hardwareMap){
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
//        carousel = hardwareMap.get(DcMotor.class, "carousel");
//        intake = hardwareMap.get(DcMotor.class, "take");
//        arm = hardwareMap.get(DcMotor.class, "arm");
//        trigger = hardwareMap.get(Servo.class, "trigger");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void move(double xValue, double yValue){
        smoother(leftMotor, yValue - xValue);
        smoother(rightMotor, yValue + xValue);
    }

    boolean unlockCarousel = true;
    boolean activateCarousel = false;
    public void carousel(boolean trig) {

        if (trig && unlockCarousel){
            activateCarousel = !activateCarousel;
        }

        if (activateCarousel){
            smoother(carousel, 1);
        } else {
            smoother(carousel, 0);
        }

        unlockCarousel = !trig;
    }

    boolean unlockIntake = true;
    boolean activateIntake = false;
    public void intake(boolean trig) {

        if (trig && unlockIntake){
            activateCarousel = !activateCarousel;
        }

        if (activateIntake){
            smoother(intake, 1);
        } else {
            smoother(intake, 0);
        }

        unlockIntake = !trig;
    }

    public void trigger(boolean trig){
        if(trig){
            ejector.setPosition(1);
        }else{
            ejector.setPosition(0);
        }
    }

    public void arm(double power){
        smoother(arm, power);
    }

    public void smoother (@NonNull DcMotor motor, double targetPower){
        double lastPower = motor.getPower();
        if (Math.abs(lastPower - targetPower) > smoother) {
            if      (lastPower > targetPower) targetPower = lastPower - smoother;
            else                              targetPower = lastPower + smoother;
        }
        motor.setPower(targetPower);
    }
}