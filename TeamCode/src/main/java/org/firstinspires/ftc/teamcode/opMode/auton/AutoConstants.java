package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class AutoConstants {
    // GLOBAL
    public static double START_X = 36;
    public static double START_Y = 70.5 - (13.3858/2);
    public static double START_HEADING = Math.toRadians(-90);

    // BASIC PARKING

    public static double FORWARD_DIST = 30;
    public static double LATERAL_DIST = 24;

    // BLUE LEFT

    public static final double BL_PRELOAD_X = 36;
    public static final double BL_PRELOAD_Y = 0;
    public static final double BL_PRELOAD_HEADING = Math.toRadians(-90);

    public static final double BL_STACK_X = 55;
    public static final double BL_STACK_Y = 12;
    public static final double BL_STACK_HEADING = Math.toRadians(180);

    public static final Pose2d BL_START = new Pose2d(START_X, START_Y, START_HEADING);
    public static final Pose2d BL_PRELOAD = new Pose2d(BL_PRELOAD_X, BL_PRELOAD_Y, BL_PRELOAD_HEADING);
    public static final Pose2d BL_STACK = new Pose2d(BL_STACK_X, BL_STACK_Y, BL_STACK_HEADING);

}
