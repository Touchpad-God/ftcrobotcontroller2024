package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Disabled
@Autonomous
public class CenterStageUpperAutoRedAlt extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(36, 12, Math.toRadians(180)));

        TrajectorySequenceBuilder traj = drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(180)))
                .setReversed(true)
                .splineTo(new Vector2d(54, 30), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(30, 46, Math.toRadians(270)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(54, 24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, 12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(60, -48), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36, -60), Math.toRadians(270));

        drive.followTrajectorySequence(traj.build());
    }
}
