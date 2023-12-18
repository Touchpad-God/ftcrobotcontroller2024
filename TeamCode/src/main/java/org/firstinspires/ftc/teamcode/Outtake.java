package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake extends Hardware {
    public enum OuttakeState {HOME, IDLE, TRANSFERRING, INTAKING, HORIZONTALEXTEND, ZERODEGREES, SIXTYDEGREES, ONETWENTYDEGREES, ONEEIGHTYDEGREES, RETURNING, RETRACTED}
    Timer timer = new Timer();
    int outtakeOffset = 78;

    public OuttakeState outtakeState =  OuttakeState.HOME;

    @Override
    public void init() {
        super.init();
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setPosition(int outtakeNumber) {
        outtakeMotor1.setTargetPosition(-(Math.multiplyExact(outtakeNumber, outtakeOffset)+192));
        outtakeMotor2.setTargetPosition(Math.multiplyExact(outtakeNumber, outtakeOffset)+192);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor1.setPower(1);
        outtakeMotor2.setPower(1);
    }
}
