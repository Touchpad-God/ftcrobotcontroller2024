package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class CorrectAngle extends LinearOpMode {
    public SampleMecanumDrive drive;
    public IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        waitForStart();

        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (currentAngle < 0) {
            currentAngle += 360;
        }

        TrajectorySequenceBuilder traj =  drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(450-currentAngle));

        drive.followTrajectorySequence(traj.build());

    }
}
