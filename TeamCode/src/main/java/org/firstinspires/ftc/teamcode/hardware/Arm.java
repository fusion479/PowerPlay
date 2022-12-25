package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends Mechanism {

    private Servo armLeft1;
    private Servo armLeft2;
    private Servo armRight1;
    private Servo armRight2;

    public Arm(LinearOpMode opMode) {this.opMode = opMode;}

    @Override
    public void init(HardwareMap hwMap) {
        armLeft1 = hwMap.get(Servo.class, "armLeft1");
        armLeft2 = hwMap.get(Servo.class, "armLeft2");
        armRight1 = hwMap.get(Servo.class, "armRight1");
        armRight2 = hwMap.get(Servo.class, "armRight2");

        // TODO: uncomment as necessary
//        armLeft1.setDirection(Servo.Direction.REVERSE);
        armLeft2.setDirection(Servo.Direction.REVERSE);
        armRight1.setDirection(Servo.Direction.REVERSE);
//        armRight2.setDirection(Servo.Direction.REVERSE);

    }

    public void moveArm(double position) {
        armLeft1.setPosition(position);
        armLeft2.setPosition(position);
        armRight1.setPosition(position);
        armRight2.setPosition(position);
    }

    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_down) {
            moveArm(0);
        } else if (gamepad.dpad_right) {
            moveArm(.5);
        } else if (gamepad.dpad_up) {
            moveArm(1);
        }
    }
}
