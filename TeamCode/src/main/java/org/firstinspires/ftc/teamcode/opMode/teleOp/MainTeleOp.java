package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.TwoStageArm;
@TeleOp (name = "Main TeleOp", group = "_Main")
@Config
public class MainTeleOp extends LinearOpMode {
    TwoStageArm twoStageArm = new TwoStageArm();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static int target = 675;
    public static int motor = 1;
    public double powerOut;

    @Override
    public void runOpMode() throws InterruptedException {
        twoStageArm.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            twoStageArm.power(0.25 * gamepad1.left_stick_x, 1);
            twoStageArm.power(0.25 * gamepad1.right_stick_x, 2);

            drive.setWeightedDrivePower(
                    new Pose2d(

                            -gamepad2.right_stick_x,
                           -gamepad2.left_stick_x,
                            gamepad2.left_stick_y
                    )
            );

            drive.update();
            twoStageArm.setPosition(motor, target);
            powerOut = twoStageArm.update(motor);
            if(gamepad1.a) {
                twoStageArm.power(powerOut, motor);
            }
            if(gamepad1.b) {
                twoStageArm.recalibrate();
            }
            tele.addData("motor1 pos", twoStageArm.motors[0].getCurrentPosition());
            tele.addData("motor2 pos", twoStageArm.motors[1].getCurrentPosition());
            tele.addData("powerOut: ", powerOut);
            tele.addData("angle: ", twoStageArm.posToAngle(twoStageArm.motors[motor-1].getCurrentPosition()));
            tele.addData("cosine: ", Math.cos(Math.toRadians(twoStageArm.posToAngle(twoStageArm.motors[motor-1].getCurrentPosition()))));
            tele.addData("error: ", target - twoStageArm.motors[motor-1].getCurrentPosition());
            tele.addData("target: ", target);
            tele.update();
        }
    }
}
