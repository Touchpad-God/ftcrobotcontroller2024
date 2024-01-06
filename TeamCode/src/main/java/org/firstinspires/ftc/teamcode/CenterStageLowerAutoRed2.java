package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CenterStageLowerAutoRed2 extends OpMode {
    redPropLeft redPropPipeline;
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    TrajectorySequenceBuilder traj;
    TrajectorySequence driveToBackdropFromVisionLeft;
    TrajectorySequence driveToBackdropFromVisionRight;
    TrajectorySequence driveToBackdropFromVisionCenter;
    TrajectorySequence driveToBackdropReturn;
    TrajectorySequence driveToAudienceLeft;
    TrajectorySequence driveToAudienceRight;
    TrajectorySequence driveToAudienceCenter;
    static IntakeOuttakeAuto intakeOuttake;
    SampleMecanumDrive drive;

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    public static boolean parking = false;

    Timer t = new Timer();

    //vision
    private OpenCvCamera camera;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(59, 10), () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .splineToConstantHeading(new Vector2d(44, -42), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(59, -36, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(34, 48), Math.toRadians(180)).build();

        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(37.5, -37, Math.toRadians(-90)))
                .addSpatialMarker(new Vector2d(59, 10), () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineTo(new Vector2d(59, -36))
                .lineToConstantHeading(new Vector2d(59, 0))
                .splineToConstantHeading(new Vector2d(42, 46), Math.toRadians(180)).build();

        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(47.5, -47, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(59, 10), () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineToSplineHeading(new Pose2d(59, -30, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(59, 12))
                .splineToConstantHeading(new Vector2d(27, 47), Math.toRadians(180)).build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(new Pose2d(42, 46, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(new Pose2d(27, 47, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(new Pose2d(34, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .setReversed(true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(12, 12), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(34, 49.5), Math.toRadians(90)).build();

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        redPropPipeline = new redPropLeft(telemetry);
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

    }

    @Override
    public void stop() {
        intakeOuttake.stop();
        drive.imu.stop();
    }

    @Override
    public void start() {
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });
        drive.setPoseEstimate(new Pose2d(61.5, -39, Math.toRadians(0)));

        if (redPropPipeline.position == redPropLeft.PROPPOSITION.CENTER) { // center
            backdropX = 36;

            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(37.5, -39)).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceCenter);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(48).build());
                return;
            }

        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.RIGHT) { // right
            backdropX = 42;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(37.5, -37))
                    .turn(Math.toRadians(-90));

            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceRight);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(48).build());
                return;
            }

        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.LEFT) { // left
            backdropX = 30;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(47.5, -47));

            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

//            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(37.5, -36, Math.toRadians(90))).strafeRight(20.5).build());

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceLeft);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(48).build());
                return;
            }

        }

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(4, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());

        drive.followTrajectorySequence(driveToBackdropReturn);

        IntakeOuttake.outtakeTicks = 300;
        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

    }

    @Override
    public void loop() {

    }
}
