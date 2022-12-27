package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public static double on = 0;
    public static double calibrate = 1;
    public static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//
//                            -gamepad2.right_stick_x,
//                           -gamepad2.left_stick_x,
//                            gamepad2.left_stick_y
//                    )
//            );
//
//            drive.update();

            tele.update();
        }
    }
}
