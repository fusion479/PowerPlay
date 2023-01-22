package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp
@Config
public class liftTest extends LinearOpMode {
    Lift lift = new Lift();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    Arm arm = new Arm();
    public static double target = 200;
    public static double loopbool = 0;
    public static double manbool = 0;
    public static int targetMode = 0;
    public boolean isPressed = false;
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        lift.init(hardwareMap);
        arm.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm.close();
        waitForStart();
        while(opModeIsActive()) {
            arm.place();
            lift.setTargetPosition(target);
            if(loopbool == 1) {
                manbool = 0;
                lift.loop();
            }
            if(manbool == 1) {
                loopbool = 0;
                lift.setPower(gamepad1.left_stick_x);
            }
            if(loopbool != 1 && manbool != 1) {
                lift.setPower(0);
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            isPressed = gamepad1.a;
            tele.addData("liftpos: ", lift.getPos());
            tele.addData("liftpos INCHES: ", lift.getPosInches());
            tele.addData("avg error: ", lift.getAvgError());
            tele.addData("target: ", target);
            tele.addData("target INCHES: ", Lift.ticksToInches(target));
            tele.addData("error0: ", lift.getError(0));
            tele.addData("error1: ", lift.getError(1));
            tele.addData("power0: ", lift.powers[0]);
            tele.addData("power1: ", lift.powers[1]);
            tele.addData("current0: ", lift.getCurrent(0));
            tele.addData("current1: ", lift.getCurrent(1));
            tele.update();
        }
    }
}