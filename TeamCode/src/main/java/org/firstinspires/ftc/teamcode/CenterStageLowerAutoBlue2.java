package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.bluePropRight;
import org.firstinspires.ftc.teamcode.pipelines.redPropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CenterStageLowerAutoBlue2 extends OpMode {
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    TrajectorySequence driveToBackdropFromVisionLeft;
    TrajectorySequence driveToBackdropFromVisionRight;
    TrajectorySequence driveToBackdropFromVisionCenter;
    TrajectorySequence driveToBackdropReturn;
    TrajectorySequence driveToAudienceLeft;
    TrajectorySequence driveToAudienceRight;
    TrajectorySequence driveToAudienceCenter;
    TrajectorySequence driveToAudienceCycle;
    static IntakeOuttakeAuto intakeOuttake;
    bluePropRight bluePropPipeline;
    Timer t = new Timer();
    int movementOffset;

    Thread inOutThread;
    SampleMecanumDrive drive;

    public static boolean parking = true;

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    //vision
    private OpenCvCamera camera;

    Thread.UncaughtExceptionHandler h = (th, ex) -> RobotLog.ee("TEAMCODE", ex, ex.toString());

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(-37.5, -40, Math.toRadians(180)))
//                .addTemporalMarker(0.7, 0.0, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                //.splineToConstantHeading(new Vector2d(-44, -42), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-59, -36, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-59, 0), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(0)).build();

        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(-59, -38, Math.toRadians(-90)))
//                .addTemporalMarker(0.7, 0.0, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineTo(new Vector2d(-59, -36))
                .lineToConstantHeading(new Vector2d(-59, 0))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .splineToConstantHeading(new Vector2d(-41, 47), Math.toRadians(0)).build();

        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(-47.5, -47, Math.toRadians(180)))
//                .addTemporalMarker(0.7, 0.0, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineToSplineHeading(new Pose2d(-59, -30, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(-59, 12))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .splineToConstantHeading(new Vector2d(-30, 46), Math.toRadians(0)).build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(new Pose2d(-30, 46, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-10, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(new Pose2d(-41, 47, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-10, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(new Pose2d(-36, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-10, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -56, Math.toRadians(270)))
                .setReversed(true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-12, 12), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(-34, 48), Math.toRadians(90)).build();

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        bluePropPipeline = new bluePropRight(telemetry);
        camera.setPipeline(bluePropPipeline);

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

        drive.setPoseEstimate(new Pose2d(-61.5, -39, Math.toRadians(180)));

        if (bluePropPipeline.position == bluePropRight.PROPPOSITION.CENTER) { // center
            backdropX = 36;
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(-37.5, -40)).build());

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
                }
            }

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceCenter);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(36).build());
                return;
            }

        } else if (bluePropPipeline.position == bluePropRight.PROPPOSITION.RIGHT) { // right
            backdropX = 42;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-47.5, -47));

            drive.followTrajectorySequence(traj.build());

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
                }
            }

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceRight);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(36).build());
                return;
            }

        } else if (bluePropPipeline.position == bluePropRight.PROPPOSITION.LEFT) { // left
            backdropX = 30;

            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-36, -39)).turn(Math.toRadians(90)).build());

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
                }
            }

            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(-36, -39, Math.toRadians(270))).strafeRight(23).build());

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
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(36).build());
                return;
            }

        }



        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(-12, -53, Math.toRadians(270)))
                .forward(5, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5)
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
        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
