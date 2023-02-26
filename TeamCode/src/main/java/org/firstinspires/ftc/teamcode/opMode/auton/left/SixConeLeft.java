package org.firstinspires.ftc.teamcode.opMode.auton.left;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumFaster;
import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.opMode.auton.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "LEFT 6 Cone", group = "_Auto")
@Config
public class SixConeLeft extends LinearOpMode {

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

    SampleMecanumFaster drive;
    Turret turret;
    ScoreFSM score;
    FtcDashboard dashboard;

    public static final double TURRET_SCORE_ANG = 43;
    public static final double TURRET_PICK_ANG = 180;

    public static double grabDelay = -0.45;
    public static double liftALittleAfterGrabDelay = -.15;
    public static double liftAfterGrabDelay = 0.45;
    public static double turretAfterGrabDelay = 0.15;
    public static double scoreDelay = -.2;
    public static double turretAfterScoreDelay = .7;
    public static double liftHeightMod = 175;
    public static double cycleDelay = .52;

    public static final int maxCones = 6;

    public static boolean isParked = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // called on init
        drive = new SampleMecanumFaster(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret = new Turret();
        turret.init(hardwareMap);
        turret.setTargetAngle(0);

        score = new ScoreFSM();
        score.init(hardwareMap);
        score.lift.isAuto = true;

        drive.setPoseEstimate(AutoConstants.L_START);

        TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.L_START)
                .addTemporalMarker(0, () -> {
                    score.idleU();
                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .addTemporalMarker(2.4, () -> {
                    score.highGoal();
                })
                .lineToLinearHeading(AutoConstants.L_SCORE_POSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[0]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(TURRET_PICK_ANG);
                })
                .waitSeconds(cycleDelay)
                // END CONE 1 (PRELOAD)


                .lineTo(AutoConstants.L_STACK)
                .UNSTABLE_addTemporalMarkerOffset(grabDelay, () -> {
                    score.toggleClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(liftALittleAfterGrabDelay + .03, () -> {
                    score.setTargetPosition(score.lift.lift.getPos() + liftHeightMod);
                })
                .UNSTABLE_addTemporalMarkerOffset(liftAfterGrabDelay, () -> {
                    score.highGoal();
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterGrabDelay, () -> {
                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[1]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(TURRET_PICK_ANG);
                })
                .waitSeconds(cycleDelay)
                // END CONE 2

                .lineTo(AutoConstants.L_STACK)
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
                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[2]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(TURRET_PICK_ANG);
                })
                .waitSeconds(cycleDelay)
                //END CONE 3

                .lineTo(AutoConstants.L_STACK)
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
                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[3]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(TURRET_PICK_ANG);
                })
                .waitSeconds(cycleDelay)
                //END CONE 4

                .lineTo(AutoConstants.L_STACK)
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
                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                .UNSTABLE_addTemporalMarkerOffset(turretAfterScoreDelay, () -> {
                    turret.setTargetAngle(TURRET_PICK_ANG);
                })
                .waitSeconds(cycleDelay)
                // END CONE 5

                .lineTo(AutoConstants.L_STACK)
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
                    turret.setTargetAngle(TURRET_SCORE_ANG);
                })
                .lineTo(AutoConstants.L_SCORE_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(scoreDelay, () -> {
                    score.autoScore(AutoConstants.STACK_SLIDES_POSITIONS[4]);
                })
                // END CONE 6
                .build();


        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(path.end())
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0, () -> {
                    turret.setTargetAngle(0);
                    score.idleU();
                })
                .lineToLinearHeading(AutoConstants.L_PARK_LEFT)
                .back(12)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(path.end())
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0, () -> {
                    turret.setTargetAngle(0);
                    score.idleU();
                })
                .lineToLinearHeading(AutoConstants.L_PARK_MIDDLE)
                .back(12)
                .build();
//
        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(path.end())
                .setVelConstraint(AutoConstants.PARK_VEL)
                .setAccelConstraint(AutoConstants.PARK_ACCEL)
                .addTemporalMarker(0, () -> {
                    turret.setTargetAngle(0);
                    score.idleU();
                })
                .lineToLinearHeading(AutoConstants.L_PARK_RIGHT)
                .back(12)
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        score.autoCounter = 0;

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        camera.stopStreaming();
        drive.followTrajectorySequenceAsync(path);

        lastTime = timer.milliseconds();

        while (!isStopRequested() && opModeIsActive()) {
            score.loop();
            turret.loop();
            drive.update();
            currentTime = timer.milliseconds();
            telemetry.addLine("looptime: " + (currentTime - lastTime));
            telemetry.addLine("cycle: " + score.autoCounter);
            if(score.autoCounter == maxCones + 1) {
                if (tagOfInterest == null) {
                    drive.followTrajectorySequenceAsync(middlePark);
                    isParked = true;
                } else if (tagOfInterest.id == Left) {
                    //Left Code
                    drive.followTrajectorySequenceAsync(leftPark);
                    isParked = true;
                } else if (tagOfInterest.id == Right) {
                    //Right Code
                    drive.followTrajectorySequenceAsync(rightPark);
                    isParked = true;
                } else {
                    //Middle Code
                    drive.followTrajectorySequenceAsync(middlePark);
                    isParked = true;
                }
                score.autoCounter++;
            }
            lastTime = currentTime;
            telemetry.update();
        }
    }


    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
