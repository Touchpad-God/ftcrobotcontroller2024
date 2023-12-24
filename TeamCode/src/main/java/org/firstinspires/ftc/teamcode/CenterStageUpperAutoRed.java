package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
*
* 1. create a separate thread to constantly call getRunTime() and pass into IntakeOuttake
* 2.
*
* */

@Autonomous
public class CenterStageUpperAutoRed extends LinearOpMode{
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    int location;
    static double currTime;
    static IntakeOuttakeAuto intakeOuttake;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);
        Thread inOutThread = new Thread(intakeOuttake);
        Thread time = new Thread(new Time());
        time.start();
        inOutThread.start();
        intakeOuttake.setCurrTime(getRuntime());
        intakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(-25).build());

        drive.setPoseEstimate(new Pose2d(36, 12, Math.toRadians(0)));

        location = 3;

        if (location == 1) { // center
            traj = drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .lineToSplineHeading(new Pose2d(36, 48, Math.toRadians(270))
//                    .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(270))
//                    .splineToConstantHeading(new Vector2d(12, -56), Math.toRadians(270))
//                    .lineToConstantHeading(new Vector2d(12, 26))
//                    .splineTo(new Vector2d(30, 48), Math.toRadians(90)
                    );
        } else if (location == 2) { // left
            traj = drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(90))
                    .lineToSplineHeading(new Pose2d(30, 48, Math.toRadians(270))
//                    .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(270))
//                    .splineToConstantHeading(new Vector2d(12, -56), Math.toRadians(270))
//                    .lineToConstantHeading(new Vector2d(12, 26))
//                    .splineTo(new Vector2d(30, 48), Math.toRadians(90)
                    );
        } else { //right
            traj = drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(-90));
            drive.followTrajectorySequence(traj.build());

            intakeOuttake.setCurrTime(getRuntime());
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(-90)))
                    .splineToConstantHeading(new Vector2d(48, 24), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(180));

            intakeOuttake.setCurrTime(getRuntime());
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

//                    .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(270))
//                    .splineToConstantHeading(new Vector2d(12, -56), Math.toRadians(270))
//                    .lineToConstantHeading(new Vector2d(12, 26))
//                    .splineTo(new Vector2d(30, 48), Math.toRadians(90)
//            .addDisplacementMarker(() -> {
//                intakeOuttake.setCurrTime(getRuntime());
//                intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
//                while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
//                    idle();
//                }
//            }

        }

        drive.followTrajectorySequence(traj.build());
        intakeOuttake.stop();
    }

}

class Time implements Runnable {
    public double nowTime, startTime;
    public Time() {
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        nowTime = System.currentTimeMillis() - startTime;
        IntakeOuttakeAuto.currTime = nowTime;
    }
}

