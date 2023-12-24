package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class yellow extends OpenCvPipeline {
    Telemetry telemetry;

    public yellow(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat blurred = new Mat();
    Size blur = new Size(3, 3);
    Mat hsv = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("Yellow pipeline selected");
        telemetry.update();

        Imgproc.GaussianBlur(input, blurred, blur, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(10, 90, 180);
        Scalar highHSV = new Scalar(30, 255, 255);
        Mat yellow = new Mat();

        Core.inRange(hsv, lowHSV, highHSV, yellow);

        return yellow;
    }
}
