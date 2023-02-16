package org.firstinspires.ftc.teamcode.opMode.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "Auto FSM Test", group = "dev")
@Config
public class FSM extends LinearOpMode {

    boolean isParked = false;

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

    ElapsedTime timer = new ElapsedTime();

    private enum AutoState {
        PRELOAD_DRIVE,
        STACK_DRIVE,
        STACK_PICK,
        POLE_DRIVE,
        SCORING,
        PARK,
    }
    AutoState autoState = AutoState.PRELOAD_DRIVE;

    TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
            .lineToLinearHeading(AutoConstants.BL_SCORE)
            .build();

    TrajectorySequence toStack = drive.trajectorySequenceBuilder(preload.end())
            .lineTo(AutoConstants.BL_STACK)
            .build();

    TrajectorySequence toPole = drive.trajectorySequenceBuilder(toStack.end())
            .lineToLinearHeading(AutoConstants.BL_SCORE)
            .build();

    TrajectorySequence leftPark = drive.trajectorySequenceBuilder(toPole.end())
            .lineToLinearHeading(AutoConstants.BL_PARK_LEFT)
            .back(12)
            .build();

    TrajectorySequence middlePark = drive.trajectorySequenceBuilder(toPole.end())
            .lineToLinearHeading(AutoConstants.BL_PARK_MIDDLE)
            .back(12)
            .build();

    TrajectorySequence rightPark = drive.trajectorySequenceBuilder(toPole.end())
            .lineToLinearHeading(AutoConstants.BL_PARK_RIGHT)
            .back(12)
            .build();

    private Thread preScoreThread;
    private Thread pickThread;

    public Runnable preScore = () -> {
        try {
            Thread.sleep(50);
            score.highGoal();
            Thread.sleep(100);
            turret.setTargetAngle(turretScoreAngle);
            Thread.sleep(600);
            score.score();
            conesScored++;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable pick = () -> {
        try {
            score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[conesScored - 1]);
            Thread.sleep(600);
            turret.setTargetAngle(turretPickAngle);
            Thread.sleep(700);
            score.toggleClaw();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

    //cone counting
    public static int conesScored = 0;
    public static int maxCones = 1;

    // turret positions
    public static double turretScoreAngle = 45;
    public static double turretPickAngle = 180;

    public static int postScoreDelay = 300;

    @Override
    public void runOpMode() throws InterruptedException {

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

        preScoreThread = new Thread(preScore);

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

        //INIT AND STARTED
        waitForStart();

        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        while(!isStopRequested() && opModeIsActive()) {

            /*
            START PRELOAD
            slides up, turret turn, drive to pole
            score
            slides down (arm back)
            turret turn
            drive to stack
            pick up
            slides up
            turret turn
            drive to pole
            score
            etc
             */

            switch(autoState) {
                case PRELOAD_DRIVE:
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(preload);
                        turret.setTargetPosition(turretScoreAngle);
                        score.highGoal();
                        timer.reset();
                        autoState = AutoState.SCORING;
                    }
                    break;
                case STACK_PICK:
                    if(!drive.isBusy()) {
                        autoState = AutoState.POLE_DRIVE;
                    }
                    break;
                case STACK_DRIVE:
                    if (conesScored > maxCones) {
                        autoState = AutoState.PARK;
                    } else if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(toStack);
                        runThread(pickThread);
                        autoState = AutoState.STACK_PICK;
                    }
                    break;
                case POLE_DRIVE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(toPole);
                        runThread(preScoreThread);
                        timer.reset();
                        autoState = AutoState.SCORING;
                    }
                    break;
                case SCORING:
                    if(!drive.isBusy()) {
                        if(timer.milliseconds() >= postScoreDelay) {
                            autoState = AutoState.STACK_DRIVE;
                        }
                    }
                    break;
                case PARK:
                    if(!drive.isBusy() && !isParked) {
                        turret.setTargetAngle(0);
                        score.idleU();
                        switch (tagOfInterest.id) {
                            case LEFT:
                                drive.followTrajectorySequenceAsync(leftPark);
                                break;
                            case RIGHT:
                                drive.followTrajectorySequenceAsync(rightPark);
                                break;
                            case MIDDLE:
                                drive.followTrajectorySequenceAsync(middlePark);
                                break;
                            default:
                                drive.followTrajectorySequenceAsync(middlePark);
                                break;
                        }
                        isParked = true;
                    }
                    break;
            }

            score.loop();
            turret.loop();
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("cones scored", conesScored);
            telemetry.addData("state", autoState);
            telemetry.addData("timer", timer.milliseconds());
            telemetry.addData("preScoreThreadAlive", preScoreThread.isAlive());
            telemetry.addData("pickThreadAlive", pickThread.isAlive());
            telemetry.addData("detected AprilTag", tagOfInterest.id);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("headingDEG", poseEstimate.getHeading() * 180 / Math.PI);
            telemetry.addData("headingRAD", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
