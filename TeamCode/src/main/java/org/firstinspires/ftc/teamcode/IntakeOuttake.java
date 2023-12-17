package org.firstinspires.ftc.teamcode;


public class IntakeOuttake extends Hardware {
    Timer timer = new Timer();
    public IntakeOuttake() {
        Intake intake = new Intake();
        Outtake outtake = new Outtake(hardwareMap);

    }
}
