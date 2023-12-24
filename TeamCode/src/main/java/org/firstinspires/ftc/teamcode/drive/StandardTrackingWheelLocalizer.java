package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Encoder.Direction;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 17.5/25.4; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double X_MUlTIPLIER = 1.03719008264 * 1.0701754386;
    public static double Y_MULTIPLIER = 1.03730364552 * 1.15097772732;
    public static double LATERAL_DISTANCE = 11.869716072167982; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 1.83; // in; offset of the lateral wheel
    public static Direction LEFT_ENCODER_DIRECTION = Direction.FORWARD;
    public static Direction RIGHT_ENCODER_DIRECTION = Direction.REVERSE;
    public static Direction FRONT_ENCODER_DIRECTION = Direction.REVERSE;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motor_leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motor_rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "motor_leftFront"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(LEFT_ENCODER_DIRECTION);
        rightEncoder.setDirection(RIGHT_ENCODER_DIRECTION);
        frontEncoder.setDirection(FRONT_ENCODER_DIRECTION);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * X_MUlTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * X_MUlTIPLIER),
                encoderTicksToInches(frontEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity() * X_MUlTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity() * X_MUlTIPLIER),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }
}
