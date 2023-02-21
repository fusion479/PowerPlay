
package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
@Config
@TeleOp (group = "prototype")
public class ArmTest extends LinearOpMode {
    public static double pos = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    Arm arm = new Arm();
    boolean isPressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if (!isPressed && gamepad1.a) {
                arm.toggle();
            }
            if (gamepad1.b) {
                arm.resetArm();
            }
            if (gamepad1.x) {
                arm.pick();
            }
            if (gamepad1.y) {
                arm.place();
            }
            if(gamepad1.dpad_right) {
                arm.right(pos);
            }
            if(gamepad1.dpad_left) {
                arm.left(1-pos);
            }
            isPressed = gamepad1.a;

            tele.addData("clawopen", arm.isOpen);
            tele.update();
        }
    }
}
