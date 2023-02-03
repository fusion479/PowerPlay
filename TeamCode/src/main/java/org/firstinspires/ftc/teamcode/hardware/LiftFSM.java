package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftFSM extends Mechanism {
    public Lift lift = new Lift();
    public static double lowerAmount = 125;
    public enum states {
        LOW,
        MID,
        HIGH,
        BOTTOM,
        CUSTOM,
        SCORING_LOWERED,
    };
    public states liftState;

    public static double customHeight;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        liftState = states.BOTTOM;
    }

    public void loop() {
        switch(liftState) {
            case LOW:
                lift.setTargetPosition(Lift.low);
                break;
            case MID:
                lift.setTargetPosition(Lift.mid);
                break;
            case HIGH:
                lift.setTargetPosition(Lift.high);
                break;
            case BOTTOM:
                lift.setTargetPosition(Lift.bottom);
                break;
            case CUSTOM:
                lift.setTargetPosition(customHeight);
                break;
            case SCORING_LOWERED:
                break;
        }
        lift.loop();
    }

    public void high() {
        liftState = states.HIGH;
    }
    public void mid() {
        liftState = states.MID;
    }
    public void low() {
        liftState = states.LOW;
    }
    public void bottom() {
        liftState = states.BOTTOM;
    }
    public void setTargetPosition(double pos) {
        customHeight = pos;
        liftState = states.CUSTOM;
    }
    public void lowerABit() {
        if (liftState == states.HIGH) {
            lift.setTargetPosition(Lift.high - lowerAmount);
        } else if (liftState == states.MID) {
            lift.setTargetPosition(Lift.mid - lowerAmount);
        } else if (liftState == states.LOW) {
            lift.setTargetPosition(Lift.low - lowerAmount);
        }
        liftState = states.SCORING_LOWERED;
    }

    public boolean targetReached() {
        return lift.isReached();
    }
}
