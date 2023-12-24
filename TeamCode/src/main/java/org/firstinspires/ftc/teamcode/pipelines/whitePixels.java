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

public class whitePixels extends OpenCvPipeline {
    Telemetry telemetry;


    public whitePixels(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat hsv = new Mat();
    Mat output = new Mat();
    Size blur = new Size(1.5, 1.5);
    double knownLength = 1.44;
    double focalLength = 370;

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("pixelArea pipeline selected");
        telemetry.update();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVwhite = new Scalar(0, 0, 210);
        Scalar highHSVwhite = new Scalar(255, 60, 255);
        Mat white = new Mat();

        Core.inRange(hsv, lowHSVwhite, highHSVwhite, white);

        Imgproc.GaussianBlur(white, white, blur, 0);

        List<MatOfPoint> contour = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(white, contour, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = contour.size() - 1; i >= 0; i--) {
            if (hierarchy.col(i).get(0, 0)[2] > 0 || hierarchy.col(i).get(0, 0)[3] < 0) {
                contour.remove(i);
            }
        }

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contour.size()];
        MatOfPoint2f[] approx = new MatOfPoint2f[contour.size()];
        List<Double> areas = new ArrayList<>();

        Rect[] boundRect = new Rect[contour.size()];

        for(int i = 0; i < contour.size(); i++){
            contoursPoly[i] = new MatOfPoint2f();
            approx[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contour.get(i).toArray()), contoursPoly[i], 3, true);
            double peri = Imgproc.arcLength(new MatOfPoint2f(contour.get(i).toArray()), true);
            Imgproc.approxPolyDP(new MatOfPoint2f(contour.get(i).toArray()), approx[i], 0.04 * peri, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            areas.add(Imgproc.contourArea(contour.get(i)));
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

        List<Double> distancesContour = new ArrayList<>();

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001) && approx[i].size().height > 4 && boundRect[i].area() < (avgArea * 10)){
                Imgproc.rectangle(output, boundRect[i], new Scalar(250, 100, 200));
                Imgproc.circle(output, new Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2), 0, new Scalar(0, 200, 200));
                hexagonCount++;
                double calcHeight = Math.pow(areas.get(i)*2/(3*Math.pow(3, 0.5)), 0.5)*2;
                distancesContour.add(Math.round(distanceFromCamera(knownLength, focalLength, calcHeight)*100)/100.0);
            }
        }
        telemetry.addData("hexagons", hexagonCount);
        telemetry.addData("distance (contour area) (in.)", distancesContour);
        telemetry.update();

        return output;
    }

    public double distanceFromCamera(double knownLength, double fL, double perLength){
        return((knownLength*fL)/perLength);
    }
}