package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;
import kotlin.math.MathKt;

public class Cycles {

    public static double startX = -36;
    public static double startY = -70.5 + (13.3858/2);
    public static double startAng = Math.toRadians(90);

    public static void main(String[] args) {

        Pose2d startPosRedL = new Pose2d(startX, startY, startAng);
        Pose2d startPosRedR = new Pose2d(-startX, startY, startAng);
        Pose2d startPosBlueR = new Pose2d(startX, -startY, -startAng);
        Pose2d startPosBlueL = new Pose2d(-startX, -startY, -startAng);

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.5689)
                .setDimensions(13.3858, 13.3858)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosRedL)
                                .lineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(45)))
                                .addDisplacementMarker(() -> {
                                    //lift.goHigh();
                                    //claw.open();
                                    //lift.goBottom();
                                })
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
