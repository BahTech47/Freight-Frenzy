package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ShippingElementDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "IntermediateAuto")
public class FFAuto extends LinearOpMode {

    //Smother's Array
    private final double[] force = new double[2];
    private final double[] lastForce = new double[2];

    //Encoder constants
    final int TICKS_REV = 1120; //CHANGE THIS!
    final double GEAR_REDUCTION = 0.025; //CHANGE THIS!
    final double WHEEL_CIRCUMFERENCE_CM = Math.PI * 9; //CHANGE THIS!

    //PID variables
    double kp = 1;
    double ki = 0;
    double kd = 0;
    final double k = 35;
    final long updateRate = 75L;

    //IMU variables
    BNO055IMU imu;
    final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    //Utility variables to define start position and side of the field
    boolean blueSide = gamepad1.x;
    boolean redSide = gamepad1.b;
    boolean pos1 = gamepad1.y;

    OpenCvWebcam webcam;

    DcMotor leftMotor;
    DcMotor rightMotor;

    ColorSensor colorSensor;

    //Timer object
    ElapsedTime runtime = new ElapsedTime();

    //Recognition constructor class
    ShippingElementDetector detector = new ShippingElementDetector(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.setPipeline(detector);

        initIMU();

        //Camera config
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                //Camera range and position config
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        //Motors definition
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");

        //Set motors zero power behavior
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Press Play to start!");
        telemetry.update();

        while (opModeIsActive()) {
            //get the position of the shipping element
            ShippingElementDetector.ShippingElementLocation elementLocation = detector.getShippingElementLocation();
            telemetry.addData("element location:", elementLocation);
            telemetry.update();

            //Blue side calling methods
            if(blueSide) {
                switch (elementLocation) {
                    case LEFT:
                        if (pos1) {
                            blueLeftFirstPos();
                        } else {
                            blueLeftSecondPos();
                        }
                        break;
                    case MIDDLE:
                        if (pos1) {
                            blueMidFirstPos();
                        } else {
                            blueMidSecondPos();
                        }
                        break;
                    case RIGHT:
                        if (pos1) {
                            blueRightFirstPos();
                        } else {
                            blueRightSecondPos();
                        }
                        break;
                    default:
                        telemetry.addData("Something went wrong!", "Try it again");
                }
            }

            //Red side calling methods
            else if(redSide) {
                switch (elementLocation) {
                    case LEFT:
                        if (pos1) {
                            redLeftFirstPos();
                        } else {
                            redLeftSecondPos();
                        }
                        break;
                    case MIDDLE:
                        if (pos1) {
                            redMidFirstPos();
                        } else {
                            redMidSecondPos();
                        }
                        break;
                    case RIGHT:
                        if (pos1) {
                            redRightFirstPos();
                        } else {
                            redRightSecondPos();
                        }
                        break;
                    default:
                        telemetry.addData("Something went wrong!", "Try it again");
                }
            }
            sleep(150);
        }
    }

    public void movePID(double distance, double speed){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double currentAngle;
        final double smoother = 0.2;

        // define the PD variables
        double p;
        double i = 0;
        double d;
        double pid;
        double error;
        double lastError = 0;

        // Convert the encoder values to centimeters
        double rotation = (distance * 0.025 / WHEEL_CIRCUMFERENCE_CM) / GEAR_REDUCTION;
        int targetEncoder = (int)(rotation * TICKS_REV);

        resetEncoder();

        leftMotor.setTargetPosition(targetEncoder);
        rightMotor.setTargetPosition(targetEncoder);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.01);
        rightMotor.setPower(0.01);

        while ( leftMotor.isBusy() || rightMotor.isBusy()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // the PID in action
            error = angle - currentAngle;
            p = error * kp;
            i += error * ki;
            d = (error - lastError) * kd;
            pid = (p + i + d) / k;

            force[0] =   speed - pid;
            force[1] = - speed + pid;

            // The smoother
            for (int s = 0; s <= 1; s++) {
                if (Math.abs(lastForce[s] - force[s]) > smoother) {
                    if      (lastForce[s] > force[s]) force[s] = lastForce[s] - smoother;
                    else                              force[s] = lastForce[s] + smoother;
                }
            }
            // Save the used force in variables to get the difference
            lastForce[0] = force[0];
            lastForce[1] = force[1];

            leftMotor.setPower( force[0] );
            rightMotor.setPower( force[1] );

            sleep(updateRate);
            lastError = error;
        }

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void resetEncoder(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initIMU (){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void turn (double target_angle, boolean is_right, double pidk) {

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

    // BLUE SIDE METHODS
    public void blueLeftFirstPos() {
        movePID(20, 1);
        turn(90, true, 1);
        movePID(58, 1);
        turn(90, false, 1);
        movePID(20, 1);

        turn(90, true, 1);
        movePID(-100, 1);
        movePID(100, 1);
        turn(90, false, 1);

        turn(90, true, 1);
        movePID(-130, 1);
    }
    public void blueLeftSecondPos() { }

    public void blueMidFirstPos() { }
    public void blueMidSecondPos() { }

    public void blueRightFirstPos() { }
    public void blueRightSecondPos() { }

    // RED SIDE METHODS
    public void redLeftFirstPos() { }
    public void redLeftSecondPos() { }

    public void redMidFirstPos() { }
    public void redMidSecondPos() { }

    public void redRightFirstPos() { }
    public void redRightSecondPos() { }
}

