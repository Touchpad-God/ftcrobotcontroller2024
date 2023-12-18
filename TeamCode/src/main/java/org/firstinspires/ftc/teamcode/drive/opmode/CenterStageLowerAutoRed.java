package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class CenterStageLowerAutoRed extends LinearOpMode {
    TrajectorySequenceBuilder traj;
    int location;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(21).build());

        drive.setPoseEstimate(new Pose2d(36, -36, Math.toRadians(0)));

        if (location == 1) { // center
            traj = drive.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(60, -24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(60, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(180)
                    );
        } else if (location == 2) { // left
            traj = drive.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(90))
                    .strafeRight(12)
                    .splineToConstantHeading(new Vector2d(60, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(180));
        } else { // right
            traj = drive.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(60, -24), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(60, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(180)
                    );
        }

        drive.followTrajectorySequence(traj.build());
    }
}
