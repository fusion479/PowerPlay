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
    public static double distanceThreshold = 37;
    public static int level = 0;
    public static double armDelay = 100;
    public enum ScoringStates {
        IDLE,
        PREP,
        SCORE
    }
    ScoringStates scoringState;
    @Override
    public void init(HardwareMap hwMap) {
        scoringState = ScoringStates.IDLE;
        lift.init(hwMap);
        arm.init(hwMap);
        clawSense = hwMap.get(DistanceSensor.class, "clawSense");
        time.reset();
    }

    public void loop() {
        switch(scoringState) {
            case IDLE:
                lift.bottom();
                arm.pick();
                if(clawSense.getDistance(DistanceUnit.MM) <= distanceThreshold) {
                    arm.close();
                    arm.place();
                    scoringState = ScoringStates.PREP;
                } else {
                    arm.open();
                }
                break;
            case PREP:
                if(clawSense.getDistance(DistanceUnit.MM) >= distanceThreshold) {
                    scoringState = ScoringStates.IDLE;
                } else {
                    lift.setHeight(level);
                    arm.close();
                    arm.place();
                }
                break;
            case SCORE:
                time.reset();
                arm.open();
                if(time.milliseconds() > armDelay) {
                    scoringState = ScoringStates.IDLE;
                }
                break;
        }
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
