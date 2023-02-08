package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Bruh", group = "_Auto")
@Config
public class Bruh extends LinearOpMode {

    SampleMecanumDrive drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    public static double turretScore = 90;
    public static double turretPick = 183;

    public static double grabDelay = -0.3;
    public static double liftALittleAfterGrabDelay = -.01;
    public static double liftAfterGrabDelay = 0.45;
    public static double turretAfterGrabDelay = 0.15;
    public static double scoreDelay = 0.2;
    public static double turretAfterScoreDelay = 1.05;
    public static double slidesAfterScoreDelay = 0.75;
    public static double liftHeightMod = 200;


    @Override
    public void runOpMode() throws InterruptedException {
        // called on init
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = new Turret();
        turret.init(hardwareMap);
        turret.setTargetAngle(0);

        score = new ScoreFSM();
        score.init(hardwareMap);
        score.lift.isAuto = true;

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(AutoConstants.BL_START);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
                .addTemporalMarker(0, () -> {
                    turret.setTargetAngle(45);
                })
                .addTemporalMarker(.8, () -> {
                    score.highGoal();
                })
                .lineToLinearHeading(new Pose2d(33, 10, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[0]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(.8)
                // END PRELOAD


                .lineTo(AutoConstants.BL_STACK)
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(48, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(.8)
                // END CYCLE 1

                .lineTo(AutoConstants.BL_STACK)
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(48, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[2]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(.8)
                //END CYCLE 2

                .lineTo(AutoConstants.BL_STACK)
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(32.45, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[3]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(.8)
                //END CYCLE 3

                .lineTo(AutoConstants.BL_STACK)
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(48, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(.8)
                //END CYCLE 4
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    score.down();
                    turret.setTargetAngle(0);
                    score.idleU();
                })
                .build();

        score.toggleClaw();

        waitForStart();
            /*
            TODO: AUTON PLANNING
            Scan AprilTag
            Drive to high goal, raise slides
            Score preload
            Drive to stack, turn turret
            Pick up cone
            Drive to high goal, arm go up, turn turret, raise slides
            Score
            Repeat
            Park with AprilTag position
             */

        drive.followTrajectorySequenceAsync(path);


        while (!isStopRequested() && opModeIsActive()) {
            score.loop();
            turret.loop();
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("headingRAD", poseEstimate.getHeading());
            telemetry.addData("headingDEG", poseEstimate.getHeading() * 180 / Math.PI);
            telemetry.update();
        }
    }
}
