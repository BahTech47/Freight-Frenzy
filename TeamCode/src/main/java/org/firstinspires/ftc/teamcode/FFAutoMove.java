package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FFAutoMove{
    // Encoder constants
    private final int TICKS_REV = 0;
    private final double GEAR_REDUCTION = 0.0;
    private final double WHEEL_CIRCUMFERENCE_CM = 0.0;

    //PID constants
    private final double kp = 1;
    private final double ki = 0;
    private final double kd = 0;
    private final double k = 35;
    private final long updateRate = 75L;

    //Smother's Array
    private final double[] force = new double[2];
    private final double[] lastForce = new double[2];

    //IMU variables
    private BNO055IMU imu;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // Motors variables
    DcMotor leftMotor;
    DcMotor rightMotor;

    // Telemetry final
    final Telemetry telemetry;

    public FFAutoMove(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void defHardware(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
    }

    public void turn (double target_angle, boolean is_right, double pidk) throws InterruptedException {

        final double threshold = 0.05;

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angle;

        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;


        if (is_right) {

            angle = - target_angle + currentAngle;
            if (angle < -180) angle += 360;
            if (angle - currentAngle > 180) currentAngle += 360;

            while (Math.abs(angle - currentAngle) > threshold) {

                telemetry.addData("imu", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - currentAngle > 180) currentAngle += 360;

                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                pid = (p + i + d) / -k;

                rightMotor.setPower(-pid * pidk);
                leftMotor.setPower( pid * pidk);

                sleep(updateRate);

                lastError = error;
            }

        } else {

            angle = target_angle + currentAngle;
            if (angle > 180) angle -= 360;
            if (currentAngle - angle > 180) currentAngle -= 360;

            while (Math.abs(angle-currentAngle) > threshold) {

                telemetry.addData("imu", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();

                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (currentAngle - angle > 180) currentAngle -= 360;

                error = angle - currentAngle;
                p = error * kp;
                i += error * ki;
                d = (error - lastError) * kd;
                pid = -(p+i+d) / k;

                rightMotor.setPower(- pid * pidk);
                leftMotor.setPower(pid * pidk);

                sleep(updateRate);

                lastError = error;
            }
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}
