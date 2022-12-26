package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import javax.sql.PooledConnection;

public class MeepMeepTesting {

    public static double startX = -36;
    public static double startY = -72;
    public static double startAng = Math.toRadians(90);

        public static void main(String[] args) {
            // Declare a MeepMeep instance
            // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPosRedL = new Pose2d(startX, startY, startAng);
        Pose2d startPosRedR = new Pose2d(-startX, startY, startAng);
        Pose2d startPosBlueR = new Pose2d(startX, -startY, -startAng);
        Pose2d startPosBlueL = new Pose2d(-startX, -startY, -startAng);

        Pose2d park1 = new Pose2d(-60, -36, startAng);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.5689)
                .setDimensions(13.3858, 13.3858)
                // Option: Set theme. Default = ColorSchemeRedDark()



                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosRedL)
                                .lineTo(new Vector2d(startX, -36))
                                .lineTo(new Vector2d(-60, -36))
                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}