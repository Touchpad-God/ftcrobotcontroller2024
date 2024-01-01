package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;

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

@Config
public class bluePropLeft extends OpenCvPipeline {
    Telemetry telemetry;

    public bluePropLeft(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat cropped;
    Mat hsv = new Mat();
    Size blur = new Size(3.0, 3.0);
    //Mat blue = new Mat();
    //Mat blue2 = new Mat();
    Mat blueCombined = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();

    public static int contourSize = 10000;

    public enum PROPPOSITION {LEFT, CENTER, RIGHT, NONE}
    public PROPPOSITION position = PROPPOSITION.NONE;

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("BluePropLeft pipeline selected");
        telemetry.update();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Size shape = input.size();

        Rect roi = new Rect(0, (int) shape.height * 1 / 3, (int) shape.width, (int) shape.height / 2);

        cropped = new Mat(hsv, roi);

        //Scalar lowerHSVblue = new Scalar(0, 55, 30);
        Scalar lowHSVblue = new Scalar(100, 55, 30);
        Scalar highHSVblue = new Scalar(130, 255, 255);
        //Scalar higherHSVblue = new Scalar(160, 255, 255);
        //TODO: Ask if we need high and low values for blue

        //Core.inRange(cropped, lowerHSVblue, lowHSVblue, blue);
        //Core.inRange(cropped, highHSVblue, higherHSVblue, blue2);
        Core.inRange(cropped, lowHSVblue, highHSVblue, blueCombined);

        //Core.bitwise_or(blue, blue2, blueCombined);

        Imgproc.GaussianBlur(blueCombined, blueCombined, blur, 0);

        Imgproc.Canny(blueCombined, edges, 100, 300);

        List<MatOfPoint> contour = new ArrayList<>();
        Imgproc.findContours(edges, contour, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        //releasing matrices
        hsv.release();
        //blue.release();
        //blue2.release();
        blueCombined.release();
        edges.release();
        hierarchy.release();
        cropped.release();

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contour.size()];
        MatOfPoint2f[] curve = new MatOfPoint2f[contour.size()];
        MatOfPoint[] contoursPolyArray = new MatOfPoint[contour.size()];

        Rect[] boundRect = new Rect[contour.size()];

        for(int i = 0; i < contour.size(); i++){
            contoursPoly[i] = new MatOfPoint2f();
            curve[i] = new MatOfPoint2f(contour.get(i).toArray());

            Imgproc.approxPolyDP(curve[i], contoursPoly[i], 3, true);

            contoursPolyArray[i] = new MatOfPoint(contoursPoly[i].toArray());

            boundRect[i] = Imgproc.boundingRect(contoursPolyArray[i]);

            //releasing matrices
            contoursPoly[i].release();
            curve[i].release();
            contoursPolyArray[i].release();
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

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001) && boundRect[i].area() < (avgArea * 10)) {
                Imgproc.rectangle(input, boundRect[i], new Scalar(250, 100, 200), 4);
            }
        }

        Imgproc.line(input, new Point(0, 380), new Point(235, 335), new Scalar(100, 100, 200), 5); //left line
        Imgproc.line(input, new Point( 440, 335), new Point(1020, 335), new Scalar(100, 100, 200), 5); //center line

        if(boundRect.length != 0){
            position = PROPPOSITION.NONE;
            onLine(boundRect);

            if(position == PROPPOSITION.NONE){
                Rect largestContour = boundRect[largestContour(boundRect)];

                int centerX = largestContour.x + largestContour.width/2;
                int centerY = largestContour.y + largestContour.height/2;

                //Imgproc.circle(input, new Point(centerX, centerY), 2, new Scalar(200, 255, 200));

                if(largestContour.area() >= contourSize){
                    propPosition(centerX);
                }else{
                    position = PROPPOSITION.RIGHT;
                }
            }
        }

        telemetry.addData("Prop position", position);

        return input;
    }

    public void propPosition(int centerX){
        if(centerX >= 0 && centerX <= 325){
            position = PROPPOSITION.LEFT;
        } else if(centerX >= 490 && centerX <= 1020){
            position = PROPPOSITION.CENTER;
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

    public void onLine(Rect[] boundRect){
        Point leftLine = new Point(180, 340);
        Point centerLine = new Point(730, 330);

        for(Rect rect: boundRect){
            if(rect.contains(leftLine)){
                position = PROPPOSITION.LEFT;
            } else if(rect.contains(centerLine)){
                position = PROPPOSITION.CENTER;
            }
        }
    }
}