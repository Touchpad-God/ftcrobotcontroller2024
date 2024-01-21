package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class IntakeOuttakeAuto extends IntakeOuttake implements Runnable {
    private volatile boolean running;
    public volatile static double startTime;
    public volatile static double currTime;
    public double pos = intakeStowed;
    public int detections;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    //    public Telemetry telemetry;

    public IntakeOuttakeAuto(HardwareMap hardwareMap) {
        super(hardwareMap);
        startTime = (double) System.currentTimeMillis() / 1000;
         aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
         visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
        this.running = true;
    }
    @Override
    public void run() {
        while (running) {
            try {
                currTime = ((double) System.currentTimeMillis() / 1000) - startTime;

                //intakePos(pos);
                outtake(currTime);
                intake(currTime);
                transfer(currTime);
                sensors();
                runTo(outtakeTicks, currTime);
                detections = aprilTag.getDetections().size();
            } catch (Exception e) {
                RobotLog.ee("TEAMCODE", e, e.toString());
            }
        }
    }

    public void intakePos(double pos) {
        intakeServo.setPosition(pos);
    }

    public void stop() {
        this.running = false;
    }
}
