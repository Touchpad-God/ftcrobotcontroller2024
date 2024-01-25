package org.firstinspires.ftc.teamcode.pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class apriltagBackdrop extends OpenCvPipeline {
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
    double tagsizeY = 0.0508;

    //positions
    public enum TAGPOSITION {LEFT, CENTER, RIGHT}
    List<TAGPOSITION> tagpositions;

    public apriltagBackdrop(Telemetry telemetry){
        this.telemetry = telemetry;
        constructMatrix();
    }

    @Override
    public Mat processFrame(Mat input){
        telemetry.addLine("apriltagBackdrop pipeline selected");
        telemetry.update();

        //list of april tags
        tagpositions = new ArrayList<>();
        List<Integer> tagIDs = new ArrayList<>();

        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        ArrayList<org.openftc.apriltag.AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        //releasing matrices
        grey.release();

        //drawing the centers for the april tags
        for(org.openftc.apriltag.AprilTagDetection detection: detections){
            Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
            drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);

            if(detection.id == 1 || detection.id == 4){
                tagpositions.add(TAGPOSITION.LEFT);
            } else if(detection.id == 2 || detection.id == 5){
                tagpositions.add(TAGPOSITION.CENTER);
            } else{
                tagpositions.add(TAGPOSITION.RIGHT);
            }

            tagIDs.add(detection.id);

            //releasing matrices
            pose.rvec.release();
            pose.tvec.release();
        }

        telemetry.addLine(String.format("%s April Tags Detected", tagpositions.size()));

        for(int i = 0; i < tagpositions.size(); i++){
            telemetry.addData("Detected tag at", tagpositions.get(i));
            telemetry.addLine(String.format("April Tag ID %s", tagIDs.get(i)));
        }
        telemetry.update();

        //Should release the camera matrix
        //cameraMatrix.release();

        return input;
    }

    public void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
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

        Imgproc.circle(buf, projectedPoints[0], thickness, new Scalar(20, 200, 255), -1);
        //releasing matrices
        axis.release();
        matProjectedPoints.release();
        coefficients.release();
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