package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class CenterStageUpperAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(36, 12, Math.toRadians(180)));

        TrajectorySequenceBuilder traj = drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(54, 30), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(30, 46, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(12, -56))
                .lineToConstantHeading(new Vector2d(12, 36))
                .splineTo(new Vector2d(30, 48), Math.toRadians(90)
                );

        drive.followTrajectorySequence(traj.build());
    }
}
