package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class CenterStageUpperAutoRed extends LinearOpMode{
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    int location;
    static double currTime;
    static IntakeOuttakeAuto intakeOuttake;

    @Override
    public void runOpMode() throws InterruptedException {
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.start();
        intakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(-24).build());

        drive.setPoseEstimate(new Pose2d(37.5, 12, Math.toRadians(0)));

        location = 2;

        if (location == 1) { // center
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .lineToSplineHeading(new Pose2d(36, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }


        } else if (location == 2) { // left
            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(90));

            drive.followTrajectorySequence(traj.build());
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(30, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else { //right
            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(-90));
            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(-90)))
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(60, 24), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(180));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }


        }


        while (opModeIsActive()) {}
        intakeOuttake.stop();
    }

}

class Time implements Runnable {
    public double startTime;
    public Time() {
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        IntakeOuttakeAuto.currTime = System.currentTimeMillis() - startTime;
    }
}

