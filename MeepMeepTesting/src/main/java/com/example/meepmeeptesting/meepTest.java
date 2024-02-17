package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        int offset = 0;
        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, -62, Math.toRadians(90)))
                                .strafeRight(37)
                                .build()
                );

        RoadRunnerBotEntity redLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.25, -62, Math.toRadians(90)))
                                .forward(50)
                                .splineToSplineHeading(new Pose2d(-38, -12, Math.toRadians(0)), Math.toRadians(0))
                                //.splineToConstantHeading(new Vector2d(-27, -10), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(35, -9.5), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(59, -10, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                //.setColorScheme(red)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(13, 62, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(13 , 32), Math.toRadians(270))
                                .back(10)
                                .splineToSplineHeading(new Pose2d(50, 37, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                //.setColorScheme()
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.25, 62, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(-37.42, 32), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-22, 8.9, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(35, 14, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(59, 12), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redLeft)
                .addEntity(redRight)
                .addEntity(blueLeft)
                .addEntity(blueRight)
                .start();
    }
}
