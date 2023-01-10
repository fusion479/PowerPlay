package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Scoring extends Mechanism {
    Lift lift = new Lift();
    Arm arm = new Arm();
    DistanceSensor clawSense;
    ElapsedTime time = new ElapsedTime();
    public static double coneDetect = 0;
    public static int level = 0;
    public static double armdelay = 100;
    public enum states {
        IDLE,
        PREP,
        SCORE
    }
    states system;
    @Override
    public void init(HardwareMap hwMap) {
        system = states.IDLE;
        lift.init(hwMap);
        arm.init(hwMap);
        clawSense = hwMap.get(DistanceSensor.class, "clawSense");
        time.reset();
    }

    public void loop() {
        switch(system) {
            case IDLE:
                lift.bottom();
                arm.pick();
                if(clawSense.getDistance(DistanceUnit.MM) <= coneDetect) {
                    arm.close();
                    arm.place();
                    system = states.PREP;
                }else {
                    arm.open();
                }
                break;
            case PREP:
                if(clawSense.getDistance(DistanceUnit.MM) >= coneDetect) {
                    system = states.IDLE;
                }else {
                    lift.setHeight(level);
                    arm.close();
                    arm.place();
                }
                break;
            case SCORE:
                time.reset();
                arm.open();
                if(time.milliseconds() > armdelay) {
                    system = states.IDLE;
                }
                break;
        }
    }

    public void idle() {
        system = states.IDLE;
    }

    public void prep() {
        system = states.PREP;
    }

    public void score() {
        system = states.SCORE;
    }

    public void setLevel(int height) {
        level = height;
    }

}
