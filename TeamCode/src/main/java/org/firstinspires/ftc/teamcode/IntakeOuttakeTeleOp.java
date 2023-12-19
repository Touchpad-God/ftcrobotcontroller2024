package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeOuttakeTeleOp {
    protected DcMotor intakeIntake;
    protected DcMotor intakeTransfer;
    protected Servo intakeServo;
    protected RevColorSensorV3 color1;
    protected RevColorSensorV3 color2;
    protected DigitalChannel beam;
    protected Telemetry telemetry;

    double intakeStowed = 0.8000;
    double intakePos1 = 0.4150;
    double intakePos2 = 0.4267;
    double intakePos3 = 0.4844;
    double intakePos4 = 0.5350;
    double intakePos5 = 0.580;
    int locationPixel = 5;
    double[] intakePositions = {intakePos1, intakePos2, intakePos3, intakePos4, intakePos5, intakeStowed};
    public static double intakePower = 1.0;
    public static double transferPower = 1.0;
    public String pixel1 = null;
    public String pixel2 = null;
    public enum IntakeState {INTAKING, BEAMNOCOLOR, BOTHCOLOR, IDLE}
    public IntakeState intakeState = IntakeState.IDLE;
    public int beambreakDetections = 0;
    public boolean beambreakPrev = true;

    public IntakeOuttakeTeleOp(HardwareMap hardwareMap) {
        intakeIntake = hardwareMap.get(DcMotor.class, "intake");
        intakeTransfer = hardwareMap.get(DcMotor.class, "transfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeLift");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        beam = hardwareMap.get(DigitalChannel.class, "beam");

        intakeIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPosition(intakeStowed);

    }

    public void update(Gamepad gamepad1) {
        if (locationPixel != 5 && gamepad1.right_trigger > 0.1) {
            if (intakeState == IntakeState.IDLE) {
                intakeState = IntakeState.INTAKING;
            }
        }
        intake();
    }

    public void intake() {
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
                sensors();
                if (pixel1 != null && pixel2 != null) {
                    intakeState = IntakeState.BOTHCOLOR;
                    intakeIntake.setPower(-intakePower);
                    intakeTransfer.setPower(transferPower);
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

    public void sensors() {
        pixel1 = null;
        pixel2 = null;
        int color1green = color1.green();
        int color1blue = color1.blue();
        int color1red = color1.red();
        int color2green = color2.green();
        int color2blue = color2.blue();
        int color2red = color2.red();
        float[] HSVValues = new float[3];
        float[] HSVValues2 = new float[3];
        Color.RGBToHSV(color1red, color1green, color1blue, HSVValues);
        if (color1.getDistance(DistanceUnit.CM) < 0.9) {
            if (HSVValues[1] > 0.5) {
                pixel1 = "white";
            } else if (HSVValues[0] > 120 && HSVValues[0] < 140) {
                pixel1 = "green";
            } else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
                pixel1 = "purple";
            } else if (HSVValues[0] > 75 && HSVValues[0] < 100) {
                pixel1 = "yellow";
            }
            Color.RGBToHSV(color2red, color2green, color2blue, HSVValues2);
            if (color2.getDistance(DistanceUnit.CM) < 0.9) {

                if (HSVValues2[0] > 75 && HSVValues2[0] < 100) {
                    pixel2 = "yellow";
                } else if (HSVValues2[0] > 120 && HSVValues2[0] < 140) {
                    pixel2 = "green";
                } else if (HSVValues2[0] > 190 && HSVValues2[0] < 220) {
                    pixel2 = "purple";
                } else if (HSVValues2[1] > 0.5) {
                    pixel2 = "white";
                }
            }
        }

//        telemetry.addData("Sensor 1 Pixel", pixel1);
//        telemetry.addData("Sensor 2 Pixel", pixel2);
//        telemetry.addData("Sensor 1 Saturation", HSVValues[1]);
//        telemetry.addData("Sensor 2 Saturation", HSVValues2[1]);
//        telemetry.update();
    }


}
