package org.firstinspires.ftc.teamcode.pipelines;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ThreadedApriltag implements Runnable {

    private static volatile boolean run;
    private static AprilTagProcessor aprilTag;

    static VisionPortal visionPortal;

    volatile static List<AprilTagDetection> currentDetections;

    public ThreadedApriltag(HardwareMap hardwareMap) {

        run = true;

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(50);
        exposureControl.setExposure( 6L , TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(250);
        sleep(20);
        visionPortal.stopLiveView();

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
        visionPortal.close();
        run = false;
    }

}
