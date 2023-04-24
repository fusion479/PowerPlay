package org.firstinspires.ftc.teamcode.opMode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(group = "_1")
public class Demo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double lastTime = timer.milliseconds();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .3,
                            -gamepad1.left_stick_x * .3,
                            -gamepad1.right_stick_x * .3
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
