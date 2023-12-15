package org.firstinspires.ftc.teamcode;

public class Timer {
    private long startTime = System.currentTimeMillis(), timeToWaitInMillis;
    private boolean readyToStartTimer = true;

    public void start(long timeToWaitInMillis){
        if (this.readyToStartTimer) {
            this.startTime = System.currentTimeMillis();
            this.timeToWaitInMillis = timeToWaitInMillis;
            this.readyToStartTimer = false;
        }
    }

    public boolean finished() {
        return System.currentTimeMillis() - startTime > timeToWaitInMillis;
    }

    public void markReady() {
        this.readyToStartTimer = true;
    }
}
