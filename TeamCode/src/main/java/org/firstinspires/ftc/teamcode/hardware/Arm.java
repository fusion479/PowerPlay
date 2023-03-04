package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends Mechanism{
    Servo left, right;
    Servo claw;
    Servo poleGuide;
    //CONSTANTS
    public static double initPos = 0.85;
    public static double pickPos = 0.155;
    public static double placePos = 0.75;
    public static double autoReady = 0.72;
    public static double lowReadyPos = 0.80;
    public double currentPos = 0;

    public static double guideScore = 0.635;
    public static double guideTurn = 0.75;
    public static double guidePick = 0.86;

    public static double close = 0.73;
    public static double open = 0.92;

    public static double distanceThreshold = 50;

    public boolean isOpen = false;
    public boolean isUp = false;
    public boolean isPlacing = false;

    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        left = hwMap.get(Servo.class, "armLeft");
        right = hwMap.get(Servo.class, "armRight");
        poleGuide = hwMap.get(Servo.class, "poleGuide");
        resetArm();
        close();
    }

    public void resetArm() {
        if(currentPos != initPos) {
            left.setPosition(initPos);
            right.setPosition(1 - initPos);
            poleGuide.setPosition(guidePick);
            isUp = true;
            currentPos = initPos;
        }
    }

    public void pick() {
        if(currentPos != pickPos) {
            left.setPosition(pickPos);
            right.setPosition(1 - pickPos);
            poleGuide.setPosition(guidePick);
            isUp = false;
            currentPos = pickPos;
        }
    }

    public void place() {
        if(currentPos != placePos) {
            left.setPosition(placePos);
            right.setPosition(1 - placePos);
            poleGuide.setPosition(guidePick);
            isUp = false;
            currentPos = placePos;
        }
    }

    public void setGuideTurn() {
        poleGuide.setPosition(guideTurn);
    }

    public void setGuideIdle() {
        poleGuide.setPosition(guidePick);
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
        right.setPosition(1 - autoReady);
        poleGuide.setPosition(guideScore);
        isUp = false;
        currentPos = autoReady;
    }

    public void lowReady() {
        left.setPosition(lowReadyPos);
        right.setPosition(1-lowReadyPos);
        isUp = false;
        currentPos = lowReadyPos;
    }
}
