package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class DriveToAprilTag extends LinearOpMode {
    public SampleMecanumDrive drive;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public static final int aprilTag_x = 29;
    public static final int aprilTag_y = 64;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        initAprilTag();

        double[] coords = aprilTagPose(4);

//        for (int i = 0; i < 100; i++) {
//            coords = aprilTagPose(4);
//        }

        telemetry.addData("x", Double.toString(coords[0]));
        telemetry.addData("y", Double.toString(coords[1]));

//        updateTelemetry(telemetry.addData());
        telemetry.update();


        drive.setPoseEstimate(new Pose2d(aprilTag_x-coords[0], aprilTag_y-coords[1], Math.toRadians(270)));

        TrajectorySequenceBuilder traj = drive.trajectorySequenceBuilder(new Pose2d(aprilTag_x-coords[0], aprilTag_y-coords[1], Math.toRadians(270 )))
                .splineTo(new Vector2d(29, 60), Math.toRadians(90));

        drive.followTrajectorySequence(traj.build());

    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    private double[] aprilTagPose(int tagID) {

        double[] coords = new double[2];

        while (aprilTag.getDetections().size() == 0) {
            continue;
        }

        for (AprilTagDetection tag:aprilTag.getDetections()) {
            if (tag.id == tagID) {
                coords[0] = tag.ftcPose.x;
                coords[1] = tag.ftcPose.y;

            }
        }

        return coords;

    }
}
