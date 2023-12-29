package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class red extends OpenCvPipeline {
    Telemetry telemetry;

    public red(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat blurred = new Mat();
    Size blur = new Size(3, 3);
    Mat hsv = new Mat();
    Mat red = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("Red pipeline selected");
        telemetry.update();

        Imgproc.GaussianBlur(input, blurred, blur, 0);

        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_HSV2BGR);

        Scalar lowHSV = new Scalar(0, 0, 100);
        Scalar highHSV = new Scalar(100, 100, 255);

        Core.inRange(hsv, lowHSV, highHSV, red);

        return red;
    }
}
