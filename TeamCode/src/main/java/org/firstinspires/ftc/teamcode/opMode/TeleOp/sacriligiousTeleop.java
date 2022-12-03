package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
@TeleOp
@Config
public class sacriligiousTeleop extends LinearOpMode {
    Arm arm = new Arm();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static double target = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            arm.power(0.25 * gamepad2.left_stick_x, 1);
            arm.power(0.1 * gamepad2.right_stick_x, 2);

            drive.setWeightedDrivePower(
                    new Pose2d(

                            -gamepad1.right_stick_x,
                           -gamepad1.left_stick_x,
                            gamepad1.left_stick_y
                    )
            );

            drive.update();
            arm.setPos(90, 1);
            if(gamepad1.a) {
                arm.update(1);
            }
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            tele.addData("actual pos: ")
            telemetry.update();
        }
    }
}
