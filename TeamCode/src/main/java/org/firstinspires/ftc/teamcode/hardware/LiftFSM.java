package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftFSM extends Mechanism {
    public Lift lift = new Lift();
    public enum states {
        low,
        mid,
        high,
        bottom,
        custom,
    };
    public states liftState;

    public static double customHeight;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        liftState = states.bottom;
    }

    public void loop() {
        switch(liftState) {
            case low:
                lift.setTargetPosition(lift.low);
                break;
            case mid:
                lift.setTargetPosition(lift.mid);
                break;
            case high:
                lift.setTargetPosition(lift.high);
                break;
            case bottom:
                lift.setTargetPosition(lift.bottom);
                break;
            case custom:
                lift.setTargetPosition(customHeight);
                break;
        }
        lift.loop();
    }

    public void high() {
        liftState = states.high;
    }
    public void mid() {
        liftState = states.mid;
    }
    public void low() {
        liftState = states.low;
    }
    public void bottom() {
        liftState = states.bottom;
    }
    public void setCustomHeight(int cone) {
        customHeight = lift.autoStack[cone];
        liftState = states.custom;
    }
}
