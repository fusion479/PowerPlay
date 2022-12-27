package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SleeveVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "Parking", group = "_Main")
@Config
public class Parking extends LinearOpMode {

    private SampleMecanumDrive drive;
    private SleeveVision sleeveVision = new SleeveVision();

    public static double FORWARD_DIST = 30;
    public static double LATERAL_DIST = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        sleeveVision.init(hardwareMap);

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .waitSeconds(0.3)
                .strafeLeft(LATERAL_DIST)
                .build();

        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_DIST)
                .waitSeconds(0.3)
                .strafeRight(LATERAL_DIST)
                .build();

        waitForStart();

        if (sleeveVision.color() == 0) {
            drive.followTrajectorySequenceAsync(middlePark);
        } else if (sleeveVision.color() == 1) {
            drive.followTrajectorySequenceAsync(leftPark);
        } else if (sleeveVision.color() == 2) {
            drive.followTrajectorySequenceAsync(middlePark);
        } else if (sleeveVision.color() == 3) {
            drive.followTrajectorySequenceAsync(rightPark);
        }

        telemetry.addData("detected region", sleeveVision.color());

        // TODO: MAKE THIS AN ENUM
        while(opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
