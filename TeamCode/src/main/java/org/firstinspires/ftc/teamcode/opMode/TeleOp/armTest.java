
package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp
public class armTest extends LinearOpMode {

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
            isPressed = gamepad1.a;
        }
    }
}
