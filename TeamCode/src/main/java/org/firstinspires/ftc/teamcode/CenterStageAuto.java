package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class CenterStageAuto extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        // relative to center of the field
        int backstageX = 36;
        int backstageY = 60;
        int backdropX = 36;
        int backdropY = -72;


        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .build()
        );


    }
}
