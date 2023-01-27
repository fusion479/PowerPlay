package org.firstinspires.ftc.teamcode.opMode.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@Autonomous(name = "FSMAuto", group = "_Auto")
@Config
public class FSM extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs (Sleeve)
    int Left = 1;
    int Middle = 2;
    int Right = 3;

    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    ElapsedTime time = new ElapsedTime();

    private enum AutoState {
        PRELOAD_DRIVE,
        SCORE,
        STACK_DRIVE,
        STACK_PICK,
        POLE_DRIVE,
        SCORING,
        PARK,
        IDLE,
    }
    AutoState autoState = AutoState.PRELOAD_DRIVE;

    TrajectorySequence preload = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
            .lineToLinearHeading(AutoConstants.BL_SCORE)
            .build();

    TrajectorySequence toStack = drive.trajectorySequenceBuilder(preload.end())
            .lineToLinearHeading(AutoConstants.BL_STACK_1)
            .build();

    TrajectorySequence toPole = drive.trajectorySequenceBuilder(toStack.end())
            .lineToLinearHeading(AutoConstants.BL_SCORE)
            .build();

    TrajectorySequence leftPark = drive.trajectorySequenceBuilder(toPole.end())
            .lineToLinearHeading(AutoConstants.BL_PARK_MIDDLE)
            .lineToLinearHeading(AutoConstants.BL_PARK_LEFT)
            .build();

    TrajectorySequence middlePark = drive.trajectorySequenceBuilder(toPole.end())
            .lineToLinearHeading(AutoConstants.BL_PARK_MIDDLE)
            .build();

    TrajectorySequence rightPark = drive.trajectorySequenceBuilder(toPole.end())
            .lineToLinearHeading(AutoConstants.BL_PARK_MIDDLE)
            .lineToLinearHeading(AutoConstants.BL_PARK_RIGHT)
            .build();

    private Thread scoreThread;

    public Runnable scoreT = () -> {
        try {
            Thread.sleep(200);
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
    //turret positions
    public static double scorePos = 125;
    public static double pickPos = 410;
    //lift positions
    public static double cone1Pos = 200;
    public static double cone2Pos = 200;
    public static double cone3Pos = 200;
    public static double cone4Pos = 200;
    public static double cone5Pos = 200;

    @Override
    public void runOpMode() throws InterruptedException {

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
                    if(tag.id == Left || tag.id == Middle || tag.id == Right) {
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

        //INIT AND STARTED
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

        while(!isStopRequested() && opModeIsActive()) {
            if(conesScored >= maxCones) {
                autoState = AutoState.PARK;
            }
            switch(autoState) {
                case PRELOAD_DRIVE:
                    if(!drive.isBusy()) {
                        //path here
                        turret.setTargetPosition(scorePos);
                        score.highGoal();
                        autoState = AutoState.SCORING;
                    }
                    break;
                case STACK_PICK:
                    if(!drive.isBusy()) {
                        //score.stepDown(height) - define this stuff later
                        score.toggleClaw();
                        autoState = AutoState.POLE_DRIVE;
                    }
                    break;
                case STACK_DRIVE:
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(toStack);
                        turret.setTargetPosition(pickPos);
                        autoState = AutoState.STACK_PICK;
                    }
                    break;
                case POLE_DRIVE:
                    if(!drive.isBusy()) {
                        //path here
                        turret.setTargetPosition(scorePos);
                        score.highGoal();
                        autoState = AutoState.SCORING;
                    }
                    break;
                case SCORING:
                    if(!drive.isBusy()) {
                        score.score();
                        autoState = AutoState.STACK_DRIVE;
                    }
                    break;
                case PARK:
                    if(!drive.isBusy()) {
                        if (tagOfInterest.id == Left) {
                            drive.followTrajectorySequenceAsync(leftPark);
                        } else if (tagOfInterest.id == Right) {
                            drive.followTrajectorySequenceAsync(rightPark);
                        } else {
                            drive.followTrajectorySequenceAsync(middlePark);
                        }
                    }
                    break;
                case IDLE:
                    //me when i am
                    break;
            }
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
