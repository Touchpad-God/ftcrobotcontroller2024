package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Intake extends Hardware {
    public enum IntakeState {INTAKING, BEAMNOCOLOR, BOTHCOLOR, IDLE}
    Timer timer = new Timer();

    private IntakeState intakeState = IntakeState.IDLE;
    private int beambreakDetections = 0;
    private boolean beambreakPrev = true;
    private final Sensors sensors = new Sensors();
    @Override
    public void loop() {
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
                intakeIntake.setPower(0);
                intakeTransfer.setPower(0);
                intakeState = IntakeState.IDLE;
                break;


        }
    }

    public void setMode(IntakeState state) {
        intakeState = state;
    }

    public IntakeState getMode() {
        return intakeState;
    }

}
