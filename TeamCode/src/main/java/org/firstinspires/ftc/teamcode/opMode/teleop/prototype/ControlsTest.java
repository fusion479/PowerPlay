package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.ScoreTurretFSM;
import org.firstinspires.ftc.teamcode.hardware.TestBot;
@TeleOp (group = "prototype")
public class ControlsTest extends LinearOpMode {
    TestBot bot = new TestBot();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            bot.run(gamepad1);
        }
        tele.addData("turr1 current: ", bot.score.turret.turrs[0].getCurrent(CurrentUnit.AMPS));
        tele.addData("turr2 current: ", bot.score.turret.turrs[1].getCurrent(CurrentUnit.AMPS));
        tele.update();
    }
}
