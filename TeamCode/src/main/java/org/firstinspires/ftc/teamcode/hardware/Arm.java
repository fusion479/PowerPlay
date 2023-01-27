package org.firstinspires.ftc.teamcode.hardware;

import android.os.UserManager;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Arm extends Mechanism{
    Servo left, right;
    Servo claw;
    DistanceSensor distanceSensor;
    //CONSTANTS
    public static double initPos = 0.98;
    public static double pickPos = 0.25;
    public static double placePos = 0.7;
            ;
    public static double close = 0.95;
    public static double open = 0.75;

    public static double distanceThreshold = 50;

    public boolean isOpen = false;
    public boolean isUp = false;
    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        left = hwMap.get(Servo.class, "armLeft");
        right = hwMap.get(Servo.class, "armRight");
        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
        resetArm();
        open();
    }

    public void resetArm() {
        left.setPosition(initPos);
        right.setPosition(1-initPos);
        isUp = true;
    }

    public void pick() {
        left.setPosition(pickPos);
        right.setPosition(1-pickPos);
        isUp = false;
    }

    public void place() {
        left.setPosition(placePos);
        right.setPosition(1-placePos);
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
        for(int i = 0; i < arr.length; i++) {
            arr[i].setPosition(pos);
        }
    }

    public boolean withinDist() {
        return distanceSensor.getDistance(DistanceUnit.MM) < distanceThreshold;
    }
}
