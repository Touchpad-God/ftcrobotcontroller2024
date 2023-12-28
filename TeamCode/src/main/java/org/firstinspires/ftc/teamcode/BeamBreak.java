package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO: Debounce this thing

public class BeamBreak implements Runnable {
    private volatile boolean status = false;
    private boolean prevStatus = false;
    private DigitalChannel beam;
    private boolean running;
    private volatile int detections;

    public BeamBreak(HardwareMap hardwareMap) {
        this.beam = hardwareMap.get(DigitalChannel.class, "beam");
        this.detections = 0;
        this.running = true;
    }

    public void run() {
        while (running) {
            this.status = beam.getState();
            if (this.status && !this.prevStatus) {
                this.detections++;
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            this.prevStatus = this.status;
        }
    }

    public int getDetections() {
        return this.detections;
    }

    public void resetDetections() {
        this.detections = 0;
    }

    public boolean getState() {
        return this.status;
    }

    public void stop() {
        this.running = false;
    }

}
