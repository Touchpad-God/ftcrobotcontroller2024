package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class yellowAndAprilTag extends OpenCvPipeline {
    Telemetry telemetry;

    //April Tag Detection Camera
    private long nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    private Mat grey = new Mat();
    private Mat cameraMatrix;
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    double tagsize = 0.0508;
    double tagsizeX = 0.0508;
    double tagsizeY = 0.0508;

    //Yellow Pixel Detection
    Mat hsv = new Mat();
    Size blur = new Size(1.5, 1.5);
    Mat hierarchy = new Mat();
    Mat yellow = new Mat();

    //positions
    public enum TAGPOSITION {LEFT, CENTER, RIGHT}
    public TAGPOSITION tagposition;
    public enum PIXELPOSITION {LEFT, RIGHT, BOTH, NONE}
    public PIXELPOSITION pixelposition;

    public yellowAndAprilTag(Telemetry telemetry){
        this.telemetry = telemetry;
        constructMatrix();
    }

    @Override
    public Mat processFrame(Mat input){
        telemetry.addLine("yellowAndAprilTag pipeline selected");
        telemetry.update();

        //position
        tagposition = TAGPOSITION.RIGHT;

        //grey for april tags, yellow for pixels
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSVyellow = new Scalar(10, 90, 180);
        Scalar highHSVyellow = new Scalar(30, 255, 255);

        Core.inRange(hsv, lowHSVyellow, highHSVyellow, yellow);

        Imgproc.GaussianBlur(yellow, yellow, blur, 0);

        //April Tag detection for the centers
        ArrayList<org.openftc.apriltag.AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        //releasing matrices
        grey.release();

        //Yellow Pixel detection for the contours
        List<MatOfPoint> contour = new ArrayList<>();

        Imgproc.findContours(yellow, contour, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = contour.size() - 1; i >= 0; i--) {
            if (hierarchy.col(i).get(0, 0)[2] > 0 || hierarchy.col(i).get(0, 0)[3] < 0) {
                contour.remove(i);
            }
        }

        //releasing matrices
        hsv.release();
        yellow.release();
        hierarchy.release();

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contour.size()];
        MatOfPoint[] contoursPolyArray = new MatOfPoint[contour.size()];
        MatOfPoint2f[] curve = new MatOfPoint2f[contour.size()];
        MatOfPoint2f[] approx = new MatOfPoint2f[contour.size()];
        MatOfPoint2f[] arcCurve = new MatOfPoint2f[contour.size()];
        MatOfPoint2f[] approxCurve = new MatOfPoint2f[contour.size()];

        Rect[] boundRect = new Rect[contour.size()];

        for(int i = 0; i < contour.size(); i++){
            contoursPoly[i] = new MatOfPoint2f();
            curve[i] = new MatOfPoint2f(contour.get(i).toArray());
            approx[i] = new MatOfPoint2f();
            arcCurve[i] = new MatOfPoint2f(contour.get(i).toArray());
            approxCurve[i] = new MatOfPoint2f(contour.get(i).toArray());

            Imgproc.approxPolyDP(curve[i], contoursPoly[i], 3, true);

            double peri = Imgproc.arcLength(arcCurve[i], true);

            contoursPolyArray[i] = new MatOfPoint(contoursPoly[i].toArray());

            Imgproc.approxPolyDP(approxCurve[i], approx[i], 0.04 * peri, true);
            boundRect[i] = Imgproc.boundingRect(contoursPolyArray[i]);

            //releasing matrices
            contoursPoly[i].release();
            contoursPolyArray[i].release();
            curve[i].release();
            arcCurve[i].release();
            approxCurve[i].release();
        }

        //drawing the centers for the april tags
        List<Point> tagCenters = new ArrayList<>();

        for(org.openftc.apriltag.AprilTagDetection detection: detections){
            Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
            Point tagCenter = drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, tagposition, detection.id, cameraMatrix);
            if(tagCenter.x != -1){
                tagCenters.add(tagCenter);
            }

            //releasing matrices
            pose.rvec.release();
            pose.tvec.release();
        }

        //drawing the boxes for the pixels
        double avgArea = pixelSizeThreshold(boundRect, input);
        List<Point> pixelCenters = new ArrayList<>();

        if(tagCenters.size() > 0){
            for(int i = 0; i != boundRect.length; i++) {
                Point pixelCenter = pixelMarker(input, boundRect[i], approx[i], avgArea, tagCenters.get(0), tagsizeX * 640);
                if (pixelCenter.x != -1) {
                    pixelCenters.add(pixelCenter);
                }

                //releasing matrices
                approx[i].release();
            }
        }

        //determining pixel position relative to tag
        try{
            pixelposition = setPixelPosition(pixelCenters, tagCenters);
            telemetry.addData("Pixel to Tag", pixelposition);
        } catch (Exception e){
            telemetry.addLine("No tags detected");
        }

        telemetry.update();

        return input;
    }

    public double pixelSizeThreshold(Rect[] boundRect, Mat input){
        double avgArea = 0;
        double total = 0;

        for(int i = 0; i != boundRect.length; i++){
            if(boundRect[i].area() > (input.width() * input.height() * 0.001)) {
                avgArea += boundRect[i].area();
                total++;
            }
        }

        return (avgArea /= total);
    }

    public Point pixelMarker(Mat input, Rect rect, MatOfPoint2f approx, double avgArea, Point tagCenter, double tagsizeX){
        if(rect.area() > (input.width() * input.height() * 0.001) && approx.size().height > 4 && rect.area() < (avgArea * 10)){
            Point pixelCenter = new Point(rect.x + rect.width/2, rect.y + rect.height/2);
            Imgproc.circle(input, pixelCenter, 0, new Scalar(0, 200, 200), 8);

            if(pixelCenter.x >= (tagCenter.x - tagsizeX) && pixelCenter.x <= (tagCenter.x + tagsizeX)){
                Imgproc.rectangle(input, rect, new Scalar(250, 25, 50), 4);
                return(pixelCenter);
            } else{
                return(new Point(-1, -1));
            }
        } else{
            return(new Point(-1, -1));
        }
    }

    public Point drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, TAGPOSITION position, int id, Mat cameraMatrix) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.

        Point tagCenter = new Point(-1, -1);

        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        MatOfDouble coefficients = new MatOfDouble();

        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, coefficients, matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        if(position == TAGPOSITION.LEFT && id == 4 || position == TAGPOSITION.LEFT && id == 1 ||
                position == TAGPOSITION.CENTER && id == 5 || position == TAGPOSITION.CENTER && id == 2 ||
                position == TAGPOSITION.RIGHT && id == 6 || position == TAGPOSITION.RIGHT && id == 3) {
            Imgproc.circle(buf, projectedPoints[0], thickness, new Scalar(255, 200, 20), -1);
            tagCenter = projectedPoints[0];
        } else{
            Imgproc.circle(buf, projectedPoints[0], thickness, new Scalar(20, 200, 255), -1);
        }

        //releasing matrices
        axis.release();
        matProjectedPoints.release();
        coefficients.release();

        return(tagCenter);
    }

    public PIXELPOSITION setPixelPosition(List<Point> pixels, List<Point> tags){
        double tagX = tags.get(0).x;

        if(pixels.size() == 1){
            if(pixels.get(0).x < tagX){
                return PIXELPOSITION.LEFT;
            } else{
                return PIXELPOSITION.RIGHT;
            }
        } else if(pixels.size() == 0){
            return PIXELPOSITION.NONE;
        } else if(pixels.size() > 2){
            return PIXELPOSITION.BOTH;
        } else {
            PIXELPOSITION current = PIXELPOSITION.NONE;
            PIXELPOSITION next = PIXELPOSITION.NONE;
            Boolean majority = false;

            for(int i = 0; i < pixels.size() - 1; i++){
                if(pixels.get(i).x < tagX){
                    current = PIXELPOSITION.LEFT;
                } else{
                    current = PIXELPOSITION.RIGHT;
                }

                if(pixels.get(i + 1).x < tagX){
                    next = PIXELPOSITION.LEFT;
                } else{
                     next = PIXELPOSITION.RIGHT;
                }

                if(current == next){
                    majority = true;
                } else{
                    majority = false;
                }
            }

            if(majority){
                return current;
            } else{
                return PIXELPOSITION.BOTH;
            }
        }
    }

    void constructMatrix()
    {
        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose){
        Pose pose = new Pose();
        pose.tvec.put(0,0, aprilTagPose.x);
        pose.tvec.put(1,0, aprilTagPose.y);
        pose.tvec.put(2,0, aprilTagPose.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.put(i,j, aprilTagPose.R.get(i,j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);

        //releasing matrices
        R.release();

        return pose;
    }

    public class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}