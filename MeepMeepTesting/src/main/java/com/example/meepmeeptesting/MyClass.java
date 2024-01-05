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

        RoadRunnerBotEntity red = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(47.5, -47, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(59, -30, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(59, 12))
                                .splineToConstantHeading(new Vector2d(42, 47), Math.toRadians(180))
                                .build()
                );
//
//        RoadRunnerBotEntity blue = new DefaultBotBuilder(meepMeep)
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, -36, Math.toRadians(0)))
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(-48, -48), Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(-36, -56, Math.toRadians(90)), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(-12, -24), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(-12, 28, Math.toRadians(90)), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(-30, 46, Math.toRadians(90)), Math.toRadians(90))
//                                .build()
//                );

//        RoadRunnerBotEntity red = new DefaultBotBuilder(meepMeep)
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(36, 12, Math.toRadians(180)))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(54, 30), Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(30, 46, Math.toRadians(270)), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(54, 24), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(60, 12), Math.toRadians(270))
//                                .splineToConstantHeading(new Vector2d(60, -48), Math.toRadians(270))
//                                .splineToConstantHeading(new Vector2d(36, -60), Math.toRadians(270))
//                                .build()
//
//                );
//        RoadRunnerBotEntity blue = new DefaultBotBuilder(meepMeep)
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(36, -36, Math.toRadians(180)))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(60, -24, Math.toRadians(270)), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(60, 24), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(90))
//                                .build()
//                );
        Image img = null;
        try { img = ImageIO.read(new File("/Users/kpeng/Downloads/field-2023-juice-dark.png")); }
        catch (IOException e) {}

//        meepMeep.setBackground(img);
        //  <following code you were using previously>

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(red)
//                .addEntity(blue)
                .start();

    }
}