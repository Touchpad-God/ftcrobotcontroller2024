package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    Trajectory driveToBackdropFromVision;
    Trajectory driveToAudience;
    Trajectory driveToBackdropReturn;
    static IntakeOuttakeAuto intakeOuttake;
    Timer t = new Timer();

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
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

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

        while(redPropPipeline.position == redPropRight.PROPPOSITION.NONE) {idle();}

        driveToBackdropReturn = drive.trajectoryBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)), true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(12, 12), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(34, 49.5), Math.toRadians(90))
                .build();

        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER) {
            backdropX = 36;
            driveToBackdropFromVision = drive.trajectoryBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(backdropX, 48, Math.toRadians(270)))
                    .build();
        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT) {
            backdropX = 42;
            driveToBackdropFromVision = drive.trajectoryBuilder(new Pose2d(34.5, 36, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(backdropX, 48, Math.toRadians(270)))
                    .build();
        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) {
            backdropX = 30;
            driveToBackdropFromVision = drive.trajectoryBuilder(new Pose2d(37.5, 12, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(backdropX, 48, Math.toRadians(270)))
                    .build();

        }

        driveToAudience = drive.trajectoryBuilder(new Pose2d(backdropX, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -51.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(61.5, 15, Math.toRadians(0)));

        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER || redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(-24, -3)).build());
        }

         if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT){ //right

            traj = drive.trajectorySequenceBuilder(new Pose2d(61.5, 15, Math.toRadians(0)))
                    .lineToConstantHeading(new Vector2d(34.5, 36))
                    .turn(Math.toRadians(90));
            drive.followTrajectorySequence(traj.build());

        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) { // left

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .turn(Math.toRadians(90));

            drive.followTrajectorySequence(traj.build());

        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
            idle();
        }

        drive.followTrajectory(driveToBackdropFromVision);

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            idle();
        }

        drive.followTrajectory(driveToAudience);

        t.start(2000);
        while (!t.finished() && IntakeOuttake.intakeState == IntakeOuttake.IntakeState.AUTOINTAKING) {
            drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
        }
        t.markReady();
        drive.setMotorPowers(0, 0, 0, 0);

        drive.followTrajectory(driveToBackdropReturn);

        IntakeOuttake.outtakeTicks = 300;
        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            idle();
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            idle();
        }

        while (opModeIsActive()) {

        }

        intakeOuttake.stop();
    }
}

