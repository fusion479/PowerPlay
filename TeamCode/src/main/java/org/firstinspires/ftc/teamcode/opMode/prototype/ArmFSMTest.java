package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ArmFSM;
@TeleOp
public class ArmFSMTest extends LinearOpMode {

    public ArmFSM arm = new ArmFSM();
    public boolean isPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a && !isPressed) {
                arm.toggleClaw();
            }
            if(gamepad1.b) {
                arm.up();
            }
            if(gamepad1.x) {
                arm.down();
            }
            if(gamepad1.y) {
                arm.ready();
            }
            isPressed = gamepad1.a;
            arm.loop();
        }
    }
}
