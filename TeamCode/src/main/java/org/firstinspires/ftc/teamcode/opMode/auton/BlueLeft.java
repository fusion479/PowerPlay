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


@Autonomous(name = "Blue Left", group = "_Auto")
@Config
public class BlueLeft extends LinearOpMode {

    SampleMecanumDrive drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    public static double turretScore = 125;
    public static double turretPick = 415;

    @Override
    public void runOpMode() throws InterruptedException {
        // called on init
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = new Turret();
        turret.init(hardwareMap);
        turret.setTargetPosition(0);

        score = new ScoreFSM();
        score.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive.setPoseEstimate(AutoConstants.BL_START);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
                .addTemporalMarker(0, () -> {
                    turret.setTargetPosition(turretScore);
                    score.highGoal();
                })
                .lineToLinearHeading(new Pose2d(36, 12, drive.getPoseEstimate().getHeading()+Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(.02, () -> {
                    turret.setTargetPosition(turretPick);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    score.idleD();
                })
                .lineTo(new Vector2d(56, 12))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(.05, () -> {
                    score.highGoal();
                    turret.setTargetPosition(turretScore);
                })
                .lineTo(new Vector2d(36, 12))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    score.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(.02, () -> {
                    turret.setTargetPosition(turretPick);
                })
                .UNSTABLE_addTemporalMarkerOffset(.2, () -> {
                    score.idleD();
                })
                .lineTo(new Vector2d(56, 12))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(.05, () -> {
                    score.highGoal();
                    turret.setTargetPosition(turretScore);
                })
                .lineTo(new Vector2d(36, 12))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    score.score();
                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    turret.setTargetPosition(turretPick);
//                })
//                .lineToLinearHeading(AutoConstants.BL_STACK_1)
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    score.toggleClaw();
//                })
//                .waitSeconds(1.5)
//                .lineToLinearHeading(AutoConstants.BL_SCORE)
//                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
//                    turret.setTargetPosition(turretScore);
//                    score.highGoal();
//                })
//                .addTemporalMarker(3, () -> {
//                    score.score();
//                })
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
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
