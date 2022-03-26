package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementDetector extends OpenCvPipeline {
    Telemetry telemetry;

    Mat mat = new Mat();

    //Detection possibilities
    public enum ShippingElementLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

    private ShippingElementLocation elementLocation;

    // Left, middle and right REGION OF INTERESTS
    // Camera resolution: 1280 x 960

    static final Rect leftROI = new Rect(
            new Point( 0, 0),
            new Point(400, 900)
    );

    static  final Rect middleROI = new Rect(
            new Point( 450, 0),
            new Point(850, 900)
    );

    static final Rect rightROI = new Rect(
            new Point(900, 0),
            new Point(1250, 900)
    );

    public ShippingElementDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // YELLOW RANGES
        Scalar lowHSV = new Scalar(15, 100, 100);
        Scalar highHSV = new Scalar(30, 255, 255);

        /* RED RANGES
        Scalar lowHSV = new Scalar(0, 70,  50);
        Scalar highHSV = new Scalar(10, 255, 255); */

        Core.inRange(mat, lowHSV, highHSV, mat);

        //Do math with ROIS's
        Mat left = mat.submat(leftROI);
        Mat middle = mat.submat(middleROI);
        Mat right = mat.submat(rightROI);

        //Define percentages
        double leftPercentage = Core.sumElems(left).val[0] / leftROI.area() / 255;
        double middlePercentage = Core.sumElems(middle).val[0] / middleROI.area() / 255;
        double rightPercentage = Core.sumElems(right).val[0] / rightROI.area() / 255;

        //Compare and define the position
        if(leftPercentage > middlePercentage && leftPercentage > rightPercentage && leftPercentage > 0.05) {
            elementLocation = ShippingElementLocation.LEFT;
        } else if(middlePercentage > leftPercentage && middlePercentage > rightPercentage && middlePercentage > 0.05) {
            elementLocation = ShippingElementLocation.MIDDLE;
        } else if(rightPercentage > leftPercentage && rightPercentage > middlePercentage && rightPercentage > 0.05) {
            elementLocation = ShippingElementLocation.RIGHT;
        }
        else {
            elementLocation = ShippingElementLocation.UNKNOWN;
        }

        //Show Percentages at the telemetry
        telemetry.addData("left percentage", Math.round(leftPercentage * 100) + "%");
        telemetry.addData("middle percentage", Math.round(middlePercentage * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightPercentage * 100) + "%");

        telemetry.update();
        return mat;
    }
    public ShippingElementLocation getShippingElementLocation() {
        return elementLocation;
    }
}
