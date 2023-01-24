package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Bruh", group = "_Auto")
public class Bruh extends LinearOpMode {
    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .build()
        );
    }
}
