package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class yellowPixels extends OpenCvPipeline {
    Telemetry telemetry;


    public yellowPixels(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat hsv = new Mat();
    Mat output = new Mat();
    Size blur = new Size(1.5, 1.5);
    double knownLength = 1.44;
    double focalLength = 370;

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("yellowPixels pipeline selected");
        telemetry.update();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVyellow = new Scalar(10, 90, 180);
        Scalar highHSVyellow = new Scalar(30, 255, 255);
        Mat yellow = new Mat();

        Core.inRange(hsv, lowHSVyellow, highHSVyellow, yellow);

        Imgproc.GaussianBlur(yellow, yellow, blur, 0);

        List<MatOfPoint> contour = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(yellow, contour, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = contour.size() - 1; i >= 0; i--) {
            if (hierarchy.col(i).get(0, 0)[2] > 0 || hierarchy.col(i).get(0, 0)[3] < 0) {
                contour.remove(i);
            }
        }

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contour.size()];
        MatOfPoint2f[] approx = new MatOfPoint2f[contour.size()];

        Rect[] boundRect = new Rect[contour.size()];

        for(int i = 0; i < contour.size(); i++){
            contoursPoly[i] = new MatOfPoint2f();
            approx[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contour.get(i).toArray()), contoursPoly[i], 3, true);
            double peri = Imgproc.arcLength(new MatOfPoint2f(contour.get(i).toArray()), true);
            Imgproc.approxPolyDP(new MatOfPoint2f(contour.get(i).toArray()), approx[i], 0.04 * peri, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        Imgproc.cvtColor(hsv, output, Imgproc.COLOR_HSV2RGB);
        int hexagonCount = 0;

        double avgArea = 0;
        double total = 0;

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001)) {
                avgArea += boundRect[i].area();
                total++;
            }
        }

        avgArea /= total;

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001) && approx[i].size().height > 4 && boundRect[i].area() < (avgArea * 10)){
                Imgproc.rectangle(output, boundRect[i], new Scalar(50, 200, 200));
                Imgproc.circle(output, new Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2), 0, new Scalar(0, 200, 200));
                hexagonCount++;
            }
        }

        telemetry.addData("hexagons", hexagonCount);
        telemetry.update();

        return output;
    }
}
