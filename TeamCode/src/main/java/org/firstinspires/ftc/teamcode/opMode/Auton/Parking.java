package org.firstinspires.ftc.teamcode.opMode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Config
public class Parking extends LinearOpMode {
    public static double time = 0.03;
    ElapsedTime timer = new ElapsedTime();
    public
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        timer.reset();
        while(opModeIsActive() && !isStopRequested()) {
            if(timer.milliseconds() < time*10) {
                bot.setWeightedDrivePower(new Pose2d(0, 1, 0));
            }
        }
    }
}
