package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Intake extends Hardware {
    public enum IntakeState {INTAKING, STOWED, BEAMNOCOLOR, BOTHCOLOR, IDLE}
    Timer timer = new Timer();
    public static double intakePower = 1.0;
    public static double transferPower = 1.0;
    public static double intakeStowed = 0.8000;
    public static double intakePos1 = 0.4178;
    public static double intakePos2 = 0.4267;
    public static double intakePos3 = 0.4844;
    public static double intakePos4 = 0.5350;
    public static double intakePos5 = 1;
    public static double locationPixel = 0;
    public static boolean dpadPressedLast = false;
    private IntakeState intakeState = IntakeState.STOWED;
    private int beambreakDetections = 0;
    private boolean beambreakPrev = true;
    private Telemetry telemetry;
    private final Sensors sensors = new Sensors();
    public void run() {
        switch(intakeState) {
            case IDLE:
                break;
            case INTAKING:
                intakeIntake.setPower(intakePower);
                intakeTransfer.setPower(transferPower);
                if (!beam.getState() && beambreakPrev) {
                    beambreakDetections++;
                }
                if (beambreakDetections >= 2) {
                    intakeState = IntakeState.BEAMNOCOLOR;
                }
                beambreakPrev = beam.getState();
                break;
            case BEAMNOCOLOR:
                intakeIntake.setPower(-intakePower);
                intakeTransfer.setPower(transferPower);
                if (sensors.getPixel1() != null && sensors.getPixel2() != null) {
                    intakeState = IntakeState.BOTHCOLOR;
                }
                break;
            case BOTHCOLOR:
                intakeServo.setPosition(intakeStowed);
                intakeState = IntakeState.STOWED;
                break;
        }
    }

}
