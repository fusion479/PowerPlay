package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism{
    Servo left, right;
    Servo claw;
    //CONSTANTS
    public static double initPos = 0.85;
    public static double pickPos = 0.155;
    public static double placePos = 0.655;
    public static double autoReady = 0.72;
    public double currentPos = 0;
            ;
    public static double close = 0.53;
    public static double open = 0.73;

    public static double distanceThreshold = 50;

    public boolean isOpen = false;
    public boolean isUp = false;
    public boolean isPlacing = false;

    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        left = hwMap.get(Servo.class, "armLeft");
        right = hwMap.get(Servo.class, "armRight");
        resetArm();
        close();
    }

    public void resetArm() {
        if(currentPos != initPos) {
            left.setPosition(initPos);
            right.setPosition(1 - initPos);
            isUp = true;
            currentPos = initPos;
        }
    }

    public void pick() {
        if(currentPos != pickPos) {
            left.setPosition(pickPos);
            right.setPosition(1 - pickPos);
            isUp = false;
            currentPos = pickPos;
        }
    }

    public void place() {
        if(currentPos != placePos) {
            left.setPosition(placePos);
            right.setPosition(1 - placePos);
            isUp = false;
            currentPos = placePos;
        }
    }

    public void open() {
        claw.setPosition(open);
        isOpen = true;
    }

    public void close() {
        claw.setPosition(close);
        isOpen = false;
    }

    public void left(double pos) {
        left.setPosition(pos);
    }

    public void right(double pos) {
        right.setPosition(pos);
    }

    public void toggle() {
        if(isOpen) {
            close();
        }else {
            open();
        }
    }

    public void toggleArm() {
        if(isUp) {
            pick();
        }else {
            resetArm();
        }
    }

    public void setPosArr(Servo[] arr, double pos) {
        for (int i = 0; i < arr.length; i++) {
            arr[i].setPosition(pos);
        }
    }

    public void setOpen(boolean bool) {
        isOpen = bool;
    }

    public void setAutoReady() {
        left.setPosition(autoReady);
        right.setPosition(1- autoReady);
        isUp = false;
        currentPos = autoReady;
    }
}
