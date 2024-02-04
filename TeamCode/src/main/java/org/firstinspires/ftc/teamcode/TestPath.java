package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(42, 48, Math.toRadians(270)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(42, 48, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(40, 32))
                .lineToConstantHeading(new Vector2d(58, 32))
                .lineToConstantHeading(new Vector2d(58, -48))
                .lineToConstantHeading(new Vector2d(36, -58), (v, pose2d, pose2d1, pose2d2) -> 12, (v, pose2d, pose2d1, pose2d2) -> 4)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(58, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58, 3), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(50, 23, Math.toRadians(300)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(40, 36))
                .splineToSplineHeading(new Pose2d(30, 48, Math.toRadians(270)), Math.toRadians(90))
                .build();

        drive.followTrajectorySequence(traj);

    }
}
