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
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name = "Preload Right", group = "_Auto")
@Config
public class RightSide extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();
    double lastTime = 0;
    double currentTime = 0;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs (Sleeve)
    final int Left = 1;
    final int Middle = 2;
    final int Right = 3;

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    public static double turretScore = 40;
    public static double turretPick = 180;

    public static double grabDelay = -0.3;
    public static double liftALittleAfterGrabDelay = -.01;
    public static double liftAfterGrabDelay = 0.45;
    public static double turretAfterGrabDelay = 0.15;
    public static double scoreDelay = 0;
    public static double turretAfterScoreDelay = .8;
    public static double slidesAfterScoreDelay = 0.75;
    public static double liftHeightMod = 175;
    public static double cycleDelay = .625;

    public static boolean isParked = false;


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

        drive.setPoseEstimate(AutoConstants.BL_START);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
                .addTemporalMarker(0, () -> {
                    score.idleU();
                    turret.setTargetAngle(-45);
                })
                .addTemporalMarker(2.4, () -> {
                    score.highGoal();
                })
                .lineToLinearHeading(new Pose2d(37.5, 12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(cycleDelay)
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
                .lineTo(new Vector2d(36.9, 11.6))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(cycleDelay)
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
                .lineTo(new Vector2d(36.8, 11.3))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[2]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(cycleDelay)
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
                .lineTo(new Vector2d(36.8, 11))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[3]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(cycleDelay)
                //END CYCLE 3

                .lineTo(AutoConstants.BL_STACK)
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay + .02, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(36.4, 10.8))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(turretPick);
                })
                .waitSeconds(cycleDelay)
                // END CYCLE 4

                .lineTo(new Vector2d(52.9, 12))
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay + .02, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(turretScore);
                })
                .lineTo(new Vector2d(36.2, 10.6))
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                //END CYCLE 5
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(new Pose2d(36.2 + 24, 10.6, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    turret.setTargetAngle(0);
                    isParked = true;
                })
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(new Pose2d(36.2, 10.6, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    turret.setTargetAngle(0);
                    isParked = true;
                })
                .build();
//
        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(new Pose2d(36.2 - 22, 10.6, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    turret.setTargetAngle(0);
                    isParked = true;
                })
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

        telemetry.setMsTransmissionInterval(50);
        score.autoCounter = 5;

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
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
        camera.stopStreaming();
        lastTime = timer.milliseconds();
        while (!isStopRequested() && opModeIsActive()) {
            score.loop();
            turret.loop();
            drive.update();
            currentTime = timer.milliseconds();
            telemetry.addLine("looptime: " + (currentTime-lastTime));
            telemetry.addLine("cycle: " + score.autoCounter);
            if(score.autoCounter == 7 && tagOfInterest != null) {
                if (tagOfInterest.id == Left) {
                    //Left Code
                    drive.followTrajectorySequenceAsync(leftPark);

                } else if (tagOfInterest.id == Right) {
                    //Right Code
                    drive.followTrajectorySequenceAsync(rightPark);
                } else {
                    drive.followTrajectorySequenceAsync(middlePark);
                }
                score.autoCounter++;
            }
            if(tagOfInterest == null && score.autoCounter == 7) {
                drive.followTrajectorySequenceAsync(middlePark);
            }
            lastTime = currentTime;
            telemetry.update();
//            if(!drive.isBusy() && !isParked) {
//                if (tagOfInterest == null) {
//                    drive.followTrajectorySequenceAsync(middlePark);
//                } else if (tagOfInterest.id == LEFT) {
//                        drive.followTrajectorySequenceAsync(leftPark);
//                    } else if (tagOfInterest.id == RIGHT) {
//                        drive.followTrajectorySequenceAsync(rightPark);
//                    } else {
//                       drive.followTrajectorySequenceAsync(middlePark);
//                    }
//                }
//                isParked = true;
        }

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("headingRAD", poseEstimate.getHeading());
//            telemetry.addData("headingDEG", poseEstimate.getHeading() * 180 / Math.PI);
    }


    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
