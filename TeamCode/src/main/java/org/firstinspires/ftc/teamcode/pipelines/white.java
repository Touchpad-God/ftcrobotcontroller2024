package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class white extends OpenCvPipeline {
    Telemetry telemetry;

    public white(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat blurred = new Mat();
    Size blur = new Size(5, 5);
    Mat hsv = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("White pipeline selected");
        telemetry.update();

        Imgproc.GaussianBlur(input, blurred, blur, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        /*
        If angled:
        Scalar lowHSV = new Scalar(0, 0, 210);
        */

        Scalar lowHSVwhite = new Scalar(0, 0, 210);
        Scalar highHSVwhite = new Scalar(255, 60, 255);
        Mat white = new Mat();

        Core.inRange(hsv, lowHSVwhite, highHSVwhite, white);

        return white;
    }
}
