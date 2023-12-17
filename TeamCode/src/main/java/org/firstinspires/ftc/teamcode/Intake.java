package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake extends Hardware {
    public enum IntakeState {INTAKING, STOWED, BEAMNOCOLOR, BOTHCOLOR, IDLE}
    Timer timer = new Timer();
    double intakeStowed = 0.8000;
    double intakePos1 = 0.4178;
    double intakePos2 = 0.4267;
    double intakePos3 = 0.4844;
    double intakePos4 = 0.5350;
    double intakePos5 = 1;
    double locationPixel = 0;
    boolean dpadPressedLast = false;
    private IntakeState intakeState = IntakeState.STOWED;
    public void run() {
        switch(intakeState) {
            case IDLE:
                break;
            case INTAKING:
                break;
        }
    }

}
