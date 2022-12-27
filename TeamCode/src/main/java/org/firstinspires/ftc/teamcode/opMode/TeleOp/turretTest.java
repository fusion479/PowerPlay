package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Turret;

@TeleOp
@Config
public class turretTest extends LinearOpMode {
    Turret turret = new Turret();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static double target = 200;
    @Override
    public void runOpMode() throws InterruptedException {
        turret.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            turret.setTargetPosition(target);
            if(gamepad1.a) {
                turret.update();
            }
            if(gamepad1.x) {
                turret.setPower(gamepad1.left_stick_x);
            }
            tele.addData("turrpos: ", turret.getPos());
            tele.addData("turrangle: ", turret.getAngle());
            tele.addData("error: ", target-turret.getPos());
            tele.addData("target: ", target);
            tele.addData("target angle: ", target/turret.tpd);
            tele.update();
        }
    }
}
