package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    private long nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    private Mat grey = new Mat();
    private Mat cameraMatrix;
    private ArrayList<org.openftc.apriltag.AprilTagDetection> detections = new ArrayList<>();
    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();

    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    double tagsize = 0.0508;
    double tagsizeX = 0.0508;
    double tagsizeY = 0.0508;

    //position
    public enum TAGPOSITION {LEFT, CENTER, RIGHT}
    public TAGPOSITION tagposition;

    public AprilTagDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
        constructMatrix();
    }

    @Override
    public Mat processFrame(Mat input){
        telemetry.addLine("AprilTagDetectionPipeline selected");
        telemetry.update();

        tagposition = TAGPOSITION.LEFT;

        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        //releasing matrices
        grey.release();

        List<Point> tagCenters = new ArrayList<>();

        for(org.openftc.apriltag.AprilTagDetection detection: detections){
            Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
            tagCenters.add(drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, tagposition, detection.id, cameraMatrix));

            //releasing matrices
            pose.rvec.release();
            pose.tvec.release();
        }

        Imgproc.line(input, new Point(0, 100), new Point(tagsizeX * 640, 100), new Scalar(255, 255, 255));

        /*
        String points = "";
        for(Point p: tagCenters){
            points += "(" + Math.round(p.x * 100.0)/100.0 + "," + Math.round(p.y * 100.0)/100.0 + ")";
        }

        telemetry.addLine(points);
        telemetry.update();
         */

        //releasing matrices
        //cameraMatrix.release();

        return input;
    }
    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

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

    public Point drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, TAGPOSITION position, int id, Mat cameraMatrix) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
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

        if(position == TAGPOSITION.LEFT && id == 4 || position == TAGPOSITION.CENTER && id == 5 || position == TAGPOSITION.RIGHT && id == 6) {
            Imgproc.circle(buf, projectedPoints[0], thickness, new Scalar(255, 200, 20), -1);
        } else{
            Imgproc.circle(buf, projectedPoints[0], thickness, new Scalar(20, 200, 255), -1);
        }

        //releasing matrices
        axis.release();
        matProjectedPoints.release();
        coefficients.release();

        return(projectedPoints[0]);
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