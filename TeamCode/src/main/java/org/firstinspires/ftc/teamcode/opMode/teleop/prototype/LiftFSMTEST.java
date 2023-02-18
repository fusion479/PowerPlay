package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LiftFSM;
@TeleOp
public class LiftFSMTEST extends LinearOpMode {

    public LiftFSM lift = new LiftFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        lift.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                lift.bottom();
            }
            if(gamepad1.b) {
                lift.low();
            }
            if(gamepad1.x) {
                lift.mid();
            }
            if(gamepad1.y) {
                lift.high();
            }
            lift.loop();
        }
    }
}
