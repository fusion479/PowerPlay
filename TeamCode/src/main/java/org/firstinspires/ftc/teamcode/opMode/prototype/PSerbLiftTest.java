package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.prototype.PSerbLift;

public class PSerbLiftTest extends LinearOpMode {
    PSerbLift lift = new PSerbLift(this);

    @Override
    public void runOpMode() {

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            if(gamepad1.a) {
                lift.extendLow();
            } else if (gamepad1.y) {
                lift.extendMedium();
            } else if (gamepad1.x) {
                lift.extendHigh();
            } else if (gamepad1.b) {
                lift.extendGround();
            }
            lift.update();
        }
    }
}
