package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class ThreadedApriltag implements Runnable {

    private volatile boolean run = true;
    private AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    volatile List<AprilTagDetection> currentDetections;

    public ThreadedApriltag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        visionPortal =  new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void run() {
        while (run) {
            currentDetections = aprilTag.getDetections();
        }
    }

    public List<AprilTagDetection> getCurrentDetections() {
        return currentDetections;
    }

    public void stop() {
        run = false;
    }

}
