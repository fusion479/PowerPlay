package org.firstinspires.ftc.teamcode.hardware;

import android.os.UserManager;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Arm extends Mechanism{
    Servo left, right;
    Servo claw;
    //CONSTANTS
    public static double initPos = 0;
    public static double pickPos = 0.5;
    public static double placePos = 1;
    public static double close = 0.95;
    public static double open = 0.75;

    public boolean isOpen = false;
    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        left = hwMap.get(Servo.class, "armLeft");
        left.setDirection(Servo.Direction.FORWARD);
        right = hwMap.get(Servo.class, "armRight");
        right.setDirection(Servo.Direction.REVERSE);
        resetArm();
        open();
    }

    public void resetArm() {
        left.setPosition(initPos);
        right.setPosition(initPos);
    }

    public void pick() {
        left.setPosition(pickPos);
        right.setPosition(pickPos);
    }

    public void place() {
        left.setPosition(placePos);
        right.setPosition(placePos);
    }

    public void open() {
        claw.setPosition(open);
        isOpen = true;
    }

    public void close() {
        claw.setPosition(close);
        isOpen = false;
    }

    public void toggle() {
        if(isOpen) {
            close();
        }else {
            open();
        }
    }

    public void setPosArr(Servo[] arr, double pos) {
        for(int i = 0; i < arr.length; i++) {
            arr[i].setPosition(pos);
        }
    }
}
