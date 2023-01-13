package org.firstinspires.ftc.teamcode.opMode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SignalSleeveWebcam;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "pserbAwardWinningPark", group = "_Main")
@Config
public class pserbPark extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    public String camera = "camera";
    private SampleMecanumDrive drive;
    private SignalSleeveWebcam signalSleeveWebcam = new SignalSleeveWebcam(this, camera);

    public static double FORWARD_DIST = -30;
    public static double LATERAL_DIST = -24;

    public static int whichPark = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        signalSleeveWebcam.init(hardwareMap);

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .waitSeconds(0.2)
                .strafeLeft(LATERAL_DIST)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .waitSeconds(0.2)
                .strafeRight(LATERAL_DIST)
                .build();


        waitForStart();


        switch (signalSleeveWebcam.side()) {
            case ONE:
                drive.followTrajectorySequenceAsync(leftPark);
                break;
            case TWO:
                drive.followTrajectorySequenceAsync(middlePark);
                break;
            case THREE:
                drive.followTrajectorySequenceAsync(rightPark);
                break;
            case NOT_FOUND:
            default:
                drive.followTrajectorySequenceAsync(middlePark);
                break;
        }

        signalSleeveWebcam.stopStreaming();

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
