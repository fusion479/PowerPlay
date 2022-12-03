package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Claw extends Mechanism {
    public Servo[] axis = new Servo[2];
    public CRServo[] spin = new CRServo[2];
    public static double downPos = 0;
    public static double upPos = 1;
    @Override
    public void init(HardwareMap hwMap) {
        axis[0] = hwMap.get(Servo.class, "axis1");
        axis[1] = hwMap.get(Servo.class, "axis2");
        spin[0] = hwMap.get(CRServo.class, "spin1");
        spin[1] = hwMap.get(CRServo.class, "spin2");
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
