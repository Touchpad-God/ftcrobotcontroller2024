package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Runnable runnable = () -> {

        };
        RoadRunnerBotEntity red = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                // Get pose from AprilTag first, no implementation yet
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(180)))
                                .addSpatialMarker(new Vector2d(-24, 56), () -> {
                                    // Intake
                                    // Use AprilTag to get pose
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(54, 30), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(30, 46, Math.toRadians(90)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(12, -56))
                                .lineToConstantHeading(new Vector2d(12, 36))
                                .splineTo(new Vector2d(30, 48), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity blue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                // Get pose from AprilTag first, no implementation yet
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -36, Math.toRadians(0)))
                                .addSpatialMarker(new Vector2d(24, 56), () -> {
                                    // Intake
                                    // Use AprilTag to get pose
                                })
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-48, -48), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(-36, -56, Math.toRadians(90)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-12, -24), Math.toRadians(90))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/kpeng/Downloads/field-2023-juice-dark.png")); }
        catch (IOException e) {}

//        meepMeep.setBackground(img);
        //  <following code you were using previously>

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(red)
                .addEntity(blue)
                .start();

    }
}