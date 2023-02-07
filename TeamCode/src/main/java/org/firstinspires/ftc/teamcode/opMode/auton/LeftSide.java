package org.firstinspires.ftc.teamcode.opMode.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "Left", group = "_Auto")
@Config
public class LeftSide extends LinearOpMode {
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

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    //Tag IDs (Sleeve)
    final int LEFT = 1;
    final int MIDDLE = 2;
    final int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    public static double turretScoreAngle = 47;
    public static double turretPickAngle = 183;

    public static double grabDelay = -0.3;
    public static double liftALittleAfterGrabDelay = 0;
    public static double liftAfterGrabDelay = 0.25;
    public static double turretAfterGrabDelay = 0.1;
    public static double scoreDelay = 0.2;
    public static double turretAfterScoreDelay = 0.75;
    public static double slidesAfterScoreDelay = 0.75;
    public static double liftHeightMod = 200;


    @Override
    public void runOpMode() throws InterruptedException {
        // called on init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

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
                .lineToLinearHeading(new Pose2d(34.5, 10.6, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[0]);
                })
                .UNSTABLE_addTemporalMarkerOffset(.075, () -> {
                    turret.setTargetAngle(turretPickAngle);
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
                    turret.setTargetAngle(turretScoreAngle);
                })
                .lineTo(new Vector2d(36, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPickAngle);
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
                    turret.setTargetAngle(turretScoreAngle);
                })
                .lineTo(new Vector2d(35, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[2]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPickAngle);
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
                    turret.setTargetAngle(turretScoreAngle);
                })
                .lineTo(new Vector2d(35, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[3]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPickAngle);
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
                    turret.setTargetAngle(49);
                })
                .lineTo(new Vector2d(34, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPickAngle);
                })
                .waitSeconds(.8)
                //END CYCLE 4

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
                    turret.setTargetAngle(49);
                })
                .lineTo(new Vector2d(34, 10))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.score();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(slidesAfterScoreDelay, () -> {
                    score.idleD();
                })
                .waitSeconds(.8)
                //END CYCLE 5
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(AutoConstants.BL_PARK_LEFT)
                .back(12)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(AutoConstants.BL_PARK_MIDDLE)
                .back(12)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(AutoConstants.BL_PARK_RIGHT)
                .back(12)
                .build();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        score.toggleClaw();

        waitForStart();

        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        drive.followTrajectorySequenceAsync(path);

        while (!isStopRequested() && opModeIsActive()) {
            score.loop();
            turret.loop();
            drive.update();
//


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("headingRAD", poseEstimate.getHeading());
            telemetry.addData("headingDEG", poseEstimate.getHeading() * 180 / Math.PI);
            telemetry.addData("lift state", score.lift.liftState);
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
