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


@Autonomous(name = "Left", group = "_Auto")
@Config
public class LeftSide extends LinearOpMode {

    SampleMecanumDrive drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    public static double turretScore = 45;
    public static double turretPick = 180;
    public static double liftAfterGrabDelay = 0.0005;
    public static double turretAfterGrabDelay = .15;
    public static double scoreDelay = .35;
    public static double turretAfterScoreDelay = .6;


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

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(AutoConstants.BL_START);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
                .addTemporalMarker(0, () -> {
                    turret.setTargetAngle(turretScore);
                    score.highGoal();
                })
                .lineToLinearHeading(new Pose2d(36, 12, drive.getPoseEstimate().getHeading()+Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(.02, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    score.idleD();
                    score.setTargetPosition(0);
                })
                // END PRELOAD


                .lineTo(new Vector2d(AutoConstants.BL_STACK_X, 12))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(36, 12))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                    score.idleD();
                    score.setTargetPosition(1);
                })
                .waitSeconds(.8)
                //END CYCLE 1


                .lineTo(new Vector2d(AutoConstants.BL_STACK_X, 12))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(36, 12))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                    score.idleD();
                    score.setTargetPosition(2);
                })
                .waitSeconds(.8)
                // END CYCLE 2
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
