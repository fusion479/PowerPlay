package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp
@Config
public class liftTest extends LinearOpMode {
    Lift lift = new Lift();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static double target = 200;
    public static double loopbool = 0;
    public static double manbool = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        lift.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
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
            tele.addData("liftpos: ", lift.getPos());
            tele.addData("avg error: ", lift.getAvgError());
            tele.addData("target: ", target);
            tele.addData("lift0pos: ", lift.getPos(0));
            tele.addData("error0: ", lift.getError(0));
            tele.addData("lift1pos: ", lift.getPos(1));
            tele.addData("error1: ", lift.getError(1));
            tele.addData("power0: ", lift.powers[0]);
            tele.addData("power1: ", lift.powers[1]);
            tele.addData("current0: ", lift.getCurrent(0));
            tele.addData("current1: ", lift.getCurrent(1));
            tele.update();
        }
    }
}