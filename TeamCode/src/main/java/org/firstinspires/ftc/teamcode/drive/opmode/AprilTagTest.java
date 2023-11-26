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

import java.util.List;

@Autonomous
public class AprilTagTest extends LinearOpMode{

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
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

        waitForStart();

//        visionPortal.stopStreaming();
//        sleep(200);
//        visionPortal.resumeStreaming();

        while (!isStopRequested()) {
            telemetry.addData("apriltags detected", Integer.toString(aprilTag.getDetections().size()));

            for (AprilTagDetection tag:aprilTag.getDetections()) {
                if (tag.id == 4) {
                    telemetry.addData("AprilTag 4 x", Double.toString(tag.ftcPose.x));
                    telemetry.addData("AprilTag 4 y", Double.toString(tag.ftcPose.y));
                }
                telemetry.update();
            }

        }
    }
}
