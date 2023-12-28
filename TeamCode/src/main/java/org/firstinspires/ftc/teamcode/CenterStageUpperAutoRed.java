package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CenterStageUpperAutoRed extends LinearOpMode{
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    static IntakeOuttakeAuto intakeOuttake;

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    //vision
    private OpenCvCamera camera;

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

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        redPropRight redPropPipeline = new redPropRight(telemetry);
        camera.setPipeline(redPropPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Camera Status: ", "Camera opened");
                telemetry.update();

                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Status: ", "Couldn't open camera");
                telemetry.update();
            }});

        waitForStart();

        if (isStopRequested()) return;

//        drive.setPoseEstimate(new Pose2d(51.5, 15, Math.toRadians(0)));

        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER || redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) {
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).lineTo(new Vector2d(-24, -3)).build());
            drive.setPoseEstimate(new Pose2d(37.5, 12, Math.toRadians(0)));
        }



        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER) { // center
            backdropX = 36;

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .waitSeconds(0.4)
                    .lineToSplineHeading(new Pose2d(backdropX, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT){ //right
            backdropX = 42;

            traj = drive.trajectorySequenceBuilder(new Pose2d(51.5, 15, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(34.5, 36, Math.toRadians(90)));
            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(34.5, 36, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(backdropX, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) { // left
            backdropX = 30;

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .turn(Math.toRadians(90));

            drive.followTrajectorySequence(traj.build());
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(backdropX, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        }

        traj = drive.trajectorySequenceBuilder(new Pose2d(backdropX, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.pos = intakeOuttake.intakePos5)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -50), Math.toRadians(270));

        drive.followTrajectorySequence(traj.build());

        intakeOuttake.intakeState = IntakeOuttake.IntakeState.INTAKING;
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(whitePixelLocation, -50, Math.toRadians(270)))
                .forward(3, (v, pose2d, pose2d1, pose2d2) -> 10, (v, pose2d, pose2d1, pose2d2) -> 10)
                .build());

        traj = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addDisplacementMarker(() -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(90));

        while (opModeIsActive()) {}
        intakeOuttake.stop();
    }
}

