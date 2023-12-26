package org.firstinspires.ftc.teamcode.pipelines;

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

public class redPropRight extends OpenCvPipeline {
    Telemetry telemetry;

    public redPropRight(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat hsv = new Mat();
    Mat output = new Mat();
    Size blur = new Size(1.5, 1.5);

    public enum PROPPOSITION {LEFT, CENTER, RIGHT, NONE}
    public PROPPOSITION position;

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("RedPropRight pipeline selected");
        telemetry.update();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVred = new Scalar(170, 140, 170);
        Scalar highHSVred = new Scalar(230, 250, 255);

        Mat red = new Mat();

        Mat edges = new Mat();

        Core.inRange(hsv, lowHSVred, highHSVred, red);

        Imgproc.GaussianBlur(red, red, blur, 0);

        Imgproc.Canny(red, edges, 100, 300);

        List<MatOfPoint> contour = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contour, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);


        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contour.size()];

        Rect[] boundRect = new Rect[contour.size()];

        for(int i = 0; i < contour.size(); i++){
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contour.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        double avgArea = 0;
        double total = 0;

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001)) {
                avgArea += boundRect[i].area();
                total++;
            }
        }

        avgArea /= total;

        List<Integer> indexes = new ArrayList<>();

        Imgproc.cvtColor(hsv, output, Imgproc.COLOR_HSV2RGB);
        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001) && boundRect[i].area() < (avgArea * 10)) {
                Imgproc.rectangle(output, boundRect[i], new Scalar(250, 100, 200));
                indexes.add(i);
            }
        }

        Imgproc.line(output, new Point( 240, 350), new Point(850, 350), new Scalar(100, 100, 200), 5); //center line
        Imgproc.line(output, new Point(1025, 350), new Point(1280, 385), new Scalar(100, 100, 200), 5); //right line

        if(boundRect.length != 0){
            Rect largestContour = boundRect[largestContour(boundRect)];

            int centerX = largestContour.x + largestContour.width/2;
            int centerY = largestContour.y + largestContour.height/2;

            Imgproc.circle(output, new Point(centerX, centerY), 2, new Scalar(200, 255, 200));

            if(largestContour.area() >= 30000){
                propPosition(centerX);
                telemetry.addData("Prop Position", position);
            }else{
                telemetry.addData("Prop Position", PROPPOSITION.LEFT);
            }
        }

        /**
         for(int i = 0; i < indexes.size(); i++){
         int centerX = boundRect[indexes.get(i)].x + boundRect[indexes.get(i)].width/2;
         int centerY = boundRect[indexes.get(i)].y + boundRect[indexes.get(i)].height/2;

         Imgproc.circle(output, new Point(centerX, centerY), 2, new Scalar(200, 255, 200));
         telemetry.addLine(propPosition(centerX));
         }
         **/

        return output;
    }

    public void propPosition(int centerX){
        if(centerX >= 240 && centerX <= 850){
            position = PROPPOSITION.CENTER;
        } else if(centerX >= 1025 && centerX <= 1280){
            position = PROPPOSITION.RIGHT;
        } else{
            position = PROPPOSITION.NONE;
        }
    }

    public int largestContour(Rect[] boundRect){
        int maxIndex = 0;
        for(int i = 0; i < boundRect.length; i++){
            if(boundRect[i].area() > boundRect[maxIndex].area()){
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}