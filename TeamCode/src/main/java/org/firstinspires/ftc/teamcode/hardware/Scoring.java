package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Scoring extends Mechanism {
    Lift lift = new Lift();
    Arm arm = new Arm();
    public DistanceSensor clawSense;
    ElapsedTime time = new ElapsedTime();
    public static double distanceThreshold = 100;
    public static int level = 0;
    public static double armDelay = 100;
    public static double raiseDelay = 100;
    public boolean isPressed = false;
    public enum ScoringStates {
        IDLE,
        PREP,
        SCORE
    }
    ScoringStates scoringState;
    public Gamepad pad;
    @Override
    public void init(HardwareMap hwMap) {
        scoringState = ScoringStates.IDLE;
        lift.init(hwMap);
        arm.init(hwMap);
        clawSense = hwMap.get(DistanceSensor.class, "clawSense");
        time.reset();
    }

    public void init(HardwareMap hwMap, Gamepad game) {
        init(hwMap);
        pad = game;
    }



    public void loop() {
        switch(scoringState) {
            case IDLE:
                lift.bottom();
                arm.pick();
                if(clawSense.getDistance(DistanceUnit.MM) <= distanceThreshold) {
                    arm.close();
                    if(time.milliseconds() > raiseDelay) {
                        arm.place();
                        scoringState = ScoringStates.PREP;
                    }
                } else {
                    arm.open();
                    time.reset();
                }
                break;
            case PREP:
                if(clawSense.getDistance(DistanceUnit.MM) >= distanceThreshold) {
                    scoringState = ScoringStates.IDLE;
                } else {
                    lift.setHeight(level);
                    arm.close();
                    arm.place();
                    time.reset();
                }
                break;
            case SCORE:
                arm.open();
                if(time.milliseconds() > armDelay) {
                    scoringState = ScoringStates.IDLE;
                    time.reset();
                }
                break;
        }
        lift.loop();
        isPressed = pad.left_bumper;
    }

    public void idle() {
        scoringState = ScoringStates.IDLE;
    }

    public void prep() {
        scoringState = ScoringStates.PREP;
    }

    public void score() {
        scoringState = ScoringStates.SCORE;
    }

    public void setLevel(int height) {
        level = height;
    }

}
