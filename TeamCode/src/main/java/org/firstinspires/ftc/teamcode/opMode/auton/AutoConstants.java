package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Mat;

@Config
public class AutoConstants {
    public static double stack2Offset = 1;

    // BLUE LEFT Q1 (+, +)
    // BLUE RIGHT Q2 (-, +)
    // RED LEFT Q3 (-, -)
    // REF RIGHT Q4 (-, +)

    // GLOBAL
    public static double START_X = 36;
    public static double START_Y = 70.5 - (13.3858/2);
    public static double START_HEADING = Math.toRadians(270);

    // LEFT PARKING VALUES
    public static double LPL_X = 12;
    public static double LPM_X = 36;
    public static double LPR_X = 60;
    // RIGHT PARKING VALUES
    public static double RPL_X = LPR_X;
    public static double RPM_X = LPM_X;
    public static double RPR_X = LPL_X;

    public static double PARK_Y = 24;
    public static double PARK_HEADING = Math.toRadians(270);

    // BASIC PARKING

    public static double FORWARD_DIST = 30;
    public static double LATERAL_DIST = 24;

    // BLUE LEFT

    public static double BL_SCORE_X = 36;
    public static double BL_SCORE_Y = 4;
    public static double BL_SCORE_HEADING = Math.toRadians(180);

    public static double BL_STACK_X = 54.9;
    public static double BL_STACK_Y = 12;
    public static double BL_STACK_HEADING = Math.toRadians(180);

    public static final Pose2d BL_START = new Pose2d(START_X, START_Y, START_HEADING);
    public static final Pose2d BL_SCORE = new Pose2d(BL_SCORE_X, BL_SCORE_Y, BL_SCORE_HEADING);
    public static final Pose2d BL_STACK_1 = new Pose2d(BL_STACK_X, BL_STACK_Y, BL_STACK_HEADING);
    public static final Pose2d BL_STACK_2 = new Pose2d(BL_STACK_X - stack2Offset, BL_STACK_Y, BL_STACK_HEADING);
    public static final Pose2d BL_PARK_LEFT = new Pose2d(LPL_X, PARK_Y, PARK_HEADING);
    public static final Pose2d BL_PARK_MIDDLE = new Pose2d(LPM_X, PARK_Y, PARK_HEADING);
    public static final Pose2d BL_PARK_RIGHT = new Pose2d(LPR_X, PARK_Y, PARK_HEADING);
}
