package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Turret;

@TeleOp (group = "prototype")
@Config
public class turretTest extends LinearOpMode {
    Turret turret = new Turret();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static double target = 0;
    public static double loopbool = 0;
    public static double manbool = 0;
    public static double calibool = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        turret.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            turret.setTargetPosition(target);
            if(loopbool == 1) {
                manbool = 0;
                turret.loop();
            }
            if(manbool == 1) {
                loopbool = 0;
                turret.setPower(0);
            }
            if(calibool == 1) {
                loopbool = 0;
                manbool = 0;
                turret.recalibrate();
                calibool = 0;
            }
            tele.addData("turr0pos: ", turret.getPos(0));
            tele.addData("turr1pos: ", turret.getPos(1));
            tele.addData("turrangle: ", turret.getAngle());
            tele.addData("error0: ", turret.getError(0));
            tele.addData("error1: ", turret.getError(1));
            tele.addData("target: ", target);
            tele.addData("target angle: ", target/turret.tpd);
            tele.addData("avg error: ", turret.getAvgError());
            tele.addData("current1: ", turret.turrs[0].getCurrent(CurrentUnit.AMPS));
            tele.addData("current2:", turret.turrs[1].getCurrent(CurrentUnit.AMPS));
            tele.update();
        }
    }
}
