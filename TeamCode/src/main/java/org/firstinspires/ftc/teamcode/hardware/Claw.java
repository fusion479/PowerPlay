package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends Mechanism {
    public Servo[] axis = new Servo[2];
    public CRServo[] spin = new CRServo[2];
    public static double downPos = 0;
    public static double upPos = 1;
    @Override
    public void init(HardwareMap hwMap) {
        axis[0] = hwMap.get(Servo.class, "axialServo1");
        axis[1] = hwMap.get(Servo.class, "axialServo2");
        spin[0] = hwMap.get(CRServo.class, "leftClaw2");
        spin[1] = hwMap.get(CRServo.class, "rightClaw2");
    }

    public void down() {
        axis[0].setPosition(downPos);
        axis[1].setPosition(downPos);
    }

    public void up() {
        axis[0].setPosition(upPos);
        axis[1].setPosition(upPos);
    }
    public void run(double power) {
        spin[0].setPower(power);
        spin[1].setPower(power);
    }
}
