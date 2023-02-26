package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.LiftFSM;
import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;
import org.firstinspires.ftc.teamcode.hardware.Turret;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double lastTime = timer.milliseconds();
        double[] driveweights = new double[]{0, 0, 0};
        while (!isStopRequested()) {
            driveweights[2] = - gamepad1.right_stick_x;
            if(Math.atan(Math.abs(-gamepad1.left_stick_x/-gamepad1.left_stick_y)) <= Math.PI/4) {
                driveweights[0] = -gamepad1.left_stick_x;
                driveweights[1] = 0;
            }else {
                driveweights[0] = 0;
                driveweights[1] = -gamepad1.left_stick_y;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            driveweights[0],
                            driveweights[1],
                            driveweights[2]
                    )
            );
            double curr = timer.milliseconds();
            telemetry.addData("loop: ", curr-lastTime);
            lastTime = curr;
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.update();
        }
    }
}
