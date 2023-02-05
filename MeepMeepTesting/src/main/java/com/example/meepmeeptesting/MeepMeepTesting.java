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

    public static double startX = 36;
    public static double startY = 70.5 + (13.3858/2);
    public static double startAng = Math.toRadians(270);

    public static double FORWARD_DIST = 30;
    public static double LATERAL_DIST = 25;

    public static double turretScoreAngle = -47;
    public static double turretPickAngle = -183;

    public static double grabDelay = -0.3;
    public static double liftALittleAfterGrabDelay = 0;
    public static double liftAfterGrabDelay = 0.25;
    public static double turretAfterGrabDelay = 0.1;
    public static double scoreDelay = 0.2;
    public static double turretAfterScoreDelay = 0.75;
    public static double slidesAfterScoreDelay = 0.75;
    public static double liftHeightMod = 200;

    public static Vector2d BR_STACK = new Vector2d(-54, 12);

        public static void main(String[] args) {
            // Declare a MeepMeep instance
            // With a field size of 800 pixels
            MeepMeep meepMeep = new MeepMeep(800);

            Pose2d startPosRedL = new Pose2d(startX, startY, startAng);
            Pose2d startPosRedR = new Pose2d(-startX, startY, startAng);
            Pose2d startPosBlueR = new Pose2d(startX, -startY, -startAng);
            Pose2d startPosBlueL = new Pose2d(-startX, -startY, -startAng);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 5.578780276476597, Math.toRadians(60), 10.5689)
                .setDimensions(13.3858, 13.3858)
                // Option: Set theme. Default = ColorSchemeRedDark()



                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosRedR)
                        .lineToLinearHeading(new Pose2d(-34.5, 10.6, Math.toRadians(0)))
                        .waitSeconds(.8)
                        // END PRELOAD


                        .lineTo(BR_STACK)
                                .lineTo(new Vector2d(-35, 10))
                        .waitSeconds(.8)
                        // END CYCLE 1

                        .lineTo(BR_STACK)
                                .lineTo(new Vector2d(-35, 10))
                        .waitSeconds(.8)
                        //END CYCLE 2

                        .lineTo(BR_STACK)
                        .lineTo(new Vector2d(-35, 10))
                        .waitSeconds(.8)
                        //END CYCLE 3

                        .lineTo(BR_STACK)
                                .lineTo(new Vector2d(-35, 10))
                        .waitSeconds(.8)
                        //END CYCLE 4

                        .lineTo(BR_STACK)
                        .lineTo(new Vector2d(-34, 10))
                        .waitSeconds(.8)
                        //END CYCLE 5
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