package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeamBreak implements Runnable {
    private volatile boolean status = False;
    private DigitalChannel beam;
    private boolean running;

    public BeamBreak(HardwareMap hardwareMap) {
        this.beam = hardwareMap.get(DigitalChannel.class, "beam");
    }

    public void run() {
        while (running) {
            this.status = beam.getState();
        }
    }

    public boolean getState() {
        return this.status;
    }

    public void stop() {
        this.running = false;
    }

}