package org.firstinspires.ftc.teamcode.opMode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SleeveVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Parking", group = "_Main")
@Config
public class Parking extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    private SampleMecanumDrive drive;
    private SleeveVision sleeveVision = new SleeveVision();

    public static double FORWARD_DIST = -30;
    public static double LATERAL_DIST = -24;

    public static int whichPark = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        sleeveVision.init(hardwareMap);

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

        tele.addData("detected region", sleeveVision.color());
        tele.update();

        waitForStart();
        int theGoal = sleeveVision.color();

        if (theGoal == 0) {
            drive.followTrajectorySequenceAsync(middlePark);
        } else if (theGoal == 1) {
            drive.followTrajectorySequenceAsync(leftPark);
        } else if (theGoal == 2) {
            drive.followTrajectorySequenceAsync(middlePark);
        } else if (theGoal == 3) {
            drive.followTrajectorySequenceAsync(rightPark);
        }

        // TODO: MAKE THIS AN ENUM
        while(opModeIsActive() && !isStopRequested()) {
            drive.update();

            tele.addData("detected region", sleeveVision.color());
            tele.update();
        }
    }
}
