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
public class bluePropRight extends OpenCvPipeline {
    Telemetry telemetry;

    public bluePropRight(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static int CORE_MEAN_VAL = 50;
    Mat cropped;
    Mat blueCombined = new Mat();
    Mat hsv = new Mat();
    Size blur = new Size(3.0, 3.0);
    Mat edges = new Mat();
    Mat hierarchy = new Mat();

    public static int contourSize = 17000;

    public static double RATIO_START = 0.0 / 6.0;
    public static double RATIO_HEIGHT = 4.0 / 10.0;

    public enum PROPPOSITION {LEFT, CENTER, RIGHT, NONE}
    public PROPPOSITION position = PROPPOSITION.NONE;

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("BluePropRight pipeline selected");
        telemetry.update();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Size shape = input.size();

        Rect roi = new Rect(0, (int) (shape.height * RATIO_START), (int) shape.width, (int) (shape.height * RATIO_HEIGHT));

        cropped = new Mat(hsv, roi);

        Imgproc.resize(cropped, cropped, new Size(), 0.5, 0.5, Imgproc.INTER_AREA);

        Scalar lowHSVblue = new Scalar(100, 55, 10);
        Scalar highHSVblue = new Scalar(130, 255, 255);

        Core.inRange(cropped, lowHSVblue, highHSVblue, blueCombined);

        Imgproc.medianBlur(blueCombined, blueCombined, 5);
        Imgproc.GaussianBlur(blueCombined, blueCombined, blur, 0);

        Imgproc.rectangle(blueCombined, new Point(0, 0), new Point(blueCombined.width(), blueCombined.height()), new Scalar(0, 0, 0), 3);

        Imgproc.Canny(blueCombined, edges, 100, 300);

        List<MatOfPoint> contour = new ArrayList<>();
        Imgproc.findContours(edges, contour, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        //releasing matrices
        hsv.release();
        //blueCombined.release();
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

            //Imgproc.approxPolyDP(curve[i], contoursPoly[i], 3, true);

            //contoursPolyArray[i] = new MatOfPoint(curve[i].toArray());

            boundRect[i] = Imgproc.boundingRect(contour.get(i));

            //releasing matrices
            contoursPoly[i].release();
            curve[i].release();
            //contoursPolyArray[i].release();
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

        //Imgproc.line(input, new Point( 300 * RATIO_HEIGHT, 110 * RATIO_HEIGHT), new Point(1050 * RATIO_HEIGHT, 110* RATIO_HEIGHT), new Scalar(100, 100, 200), 5); //center line
        //Imgproc.line(input, new Point(1260 * RATIO_HEIGHT, 120 * RATIO_HEIGHT), new Point(1500 * RATIO_HEIGHT, 385 * RATIO_HEIGHT), new Scalar(100, 100, 200), 5); //right line

        if(boundRect.length != 0){
            position = PROPPOSITION.NONE;
            int maxIndex = largestContour(boundRect, blueCombined);
            onLine(boundRect[maxIndex]);
            Rect largestContour = boundRect[maxIndex];
            //Imgproc.rectangle(input, boundRect[maxIndex], new Scalar(250, 100, 200), 2);

            if(position == PROPPOSITION.NONE){
                int centerX = largestContour.x + largestContour.width/2;
                int centerY = largestContour.y + largestContour.height/2;

                Imgproc.circle(input, new Point(centerX, centerY), 2, new Scalar(200, 255, 200), 10);

                if(largestContour.area() >= contourSize){
                    propPosition(centerX);
                }else{
                    position = PROPPOSITION.LEFT;
                }
                //Imgproc.circle(input, new Point(660 * RATIO_HEIGHT, 65 * RATIO_HEIGHT), 2, new Scalar(255, 255, 255), 10);
                //Imgproc.circle(input, new Point(1460 * RATIO_HEIGHT, 190 * RATIO_HEIGHT), 2, new Scalar(255, 255, 255), 10);
            }
        }

        telemetry.addData("Prop position", position);

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (blueCombined.width() * blueCombined.height() * 0.001) && boundRect[i].area() < (avgArea * 10)) {
                Imgproc.rectangle(blueCombined, boundRect[i], new Scalar(250, 100, 200), 2);
            }
        }

        Imgproc.circle(blueCombined, new Point(1460 * 0.5, 55), 3, new Scalar(255, 255, 255), 3);
        Imgproc.circle(blueCombined, new Point(660 * 0.5, 55), 3, new Scalar(255, 255, 255), 3);

        return blueCombined;
    }

    public void propPosition(int centerX){
        if(centerX >= 0 && centerX <= 800 * 0.5){
            position = PROPPOSITION.CENTER;
        } else if(centerX >= 850 * 0.5 && centerX <= 1280 * 0.5){
            position = PROPPOSITION.RIGHT;
        } else{
            position = PROPPOSITION.NONE;
        }
    }

    public int largestContour(Rect[] boundRect, Mat mat){
        int maxIndex = 0;
        for(int i = 0; i < boundRect.length; i++){
            cropped = new Mat(mat, boundRect[i]);
            if (Core.mean(cropped).val[0] > CORE_MEAN_VAL) {
                if(boundRect[i].area() > boundRect[maxIndex].area()){
                    maxIndex = i;
                }
            }
            //telemetry.addData("" + i, Core.mean(cropped).val[0]);
            cropped.release();
        }
        return maxIndex;
    }

    public void onLine(Rect rect){
        Point rightLine = new Point(1240 * 0.5, 40);
        Point centerLine = new Point(660 * 0.5, 40);

        if(rect.contains(rightLine)){
            position = PROPPOSITION.RIGHT;
        } else if(rect.contains(centerLine)){
            position = PROPPOSITION.CENTER;
        }
    }
}