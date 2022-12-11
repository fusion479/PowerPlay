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
    public static int target = 675;
    public static int motor = 1;
    public double powerOut;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            arm.power(0.25 * gamepad1.left_stick_x, 1);
            arm.power(0.25 * gamepad1.right_stick_x, 2);

            drive.setWeightedDrivePower(
                    new Pose2d(

                            -gamepad2.right_stick_x,
                           -gamepad2.left_stick_x,
                            gamepad2.left_stick_y
                    )
            );

            drive.update();
            arm.setPosition(motor, target);
            powerOut = arm.update(motor);
            if(gamepad1.a) {
                arm.power(powerOut, motor);
            }
            if(gamepad1.b) {
                arm.recalibrate();
            }
            tele.addData("motor1 pos", arm.motors[0].getCurrentPosition());
            tele.addData("motor2 pos", arm.motors[1].getCurrentPosition());
            tele.addData("powerOut: ", powerOut);
            tele.addData("angle: ", arm.posToAngle(arm.motors[motor-1].getCurrentPosition()));
            tele.addData("cosine: ", Math.cos(Math.toRadians(arm.posToAngle(arm.motors[motor-1].getCurrentPosition()))));
            tele.update();
        }
    }
}
