package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;

@TeleOp
public class ScoreFSMTest extends LinearOpMode {

    public ScoreFSM score = new ScoreFSM();
    public boolean isPressedx = false;
    public boolean isPressedy = false;
    public boolean isPresseda = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        score.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(!isPressedx && gamepad1.x) {
                score.toggleHigh();
            }
            if(!isPressedy && gamepad1.y) {
                score.toggleMid();
            }
            if(!isPresseda && gamepad1.a) {
                score.toggleLow();
            }
            if(!isPressedRB && gamepad1.right_bumper) {
                score.toggleClaw();
            }
            if(!isPressedLB && gamepad1.left_bumper) {
                score.toggleIdle();
            }
            isPressedRB = gamepad1.right_bumper;
            isPressedLB = gamepad1.left_bumper;
            isPresseda = gamepad1.a;
            isPressedy = gamepad1.y;
            isPressedx = gamepad1.x;
            score.loop();
        }
    }
}
