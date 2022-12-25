package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftFSM extends Mechanism {
    public LiftFSM(LinearOpMode opMode) {this.opMode = opMode;}

    public Lift lift = new Lift(opMode);

    public enum LiftState {
        EXTENDED,
        BOTTOMED,
        REST,
    };

    private Thread bottomThread;
    private Thread lowThread;
    private Thread mediumThread;
    private Thread highThread;

    // RETRACT STATE DELAY (idk)
    public static long test = 100;

    public LiftState liftState;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);

        bottomThread = new Thread(goBottom);
        lowThread = new Thread(goLow);
        mediumThread = new Thread(goMedium);
        highThread = new Thread(goHigh);
    }

    public void init(HardwareMap hwMap, Telemetry tele) {
        init(hwMap);
    }

    public Runnable goBottom = () -> {
        try {
            Thread.sleep(0);
            lift.goBottom();
            liftState = LiftState.REST;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goLow = () -> {
        try {
            lift.goLow();
            Thread.sleep(test);
            liftState = LiftState.EXTENDED;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goMedium = () -> {
        try {
            lift.goMedium();
            Thread.sleep(test);
            liftState = LiftState.EXTENDED;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goHigh = () -> {
        try {
            lift.goHigh();
            Thread.sleep(test);
            liftState = LiftState.EXTENDED;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

    // consult and change controls if needed
    public void loop(Gamepad gamepad) {
        switch (liftState) {
            case REST:
                lift.goBottom();
                liftState = LiftState.BOTTOMED;
                break;
            case BOTTOMED:
                if (gamepad.b) {
                    runThread(highThread);
                } else if (gamepad.y) {
                    runThread(mediumThread);
                } else if (gamepad.x) {
                    runThread(lowThread);
                }
                break;
            case EXTENDED:
                if (gamepad.a) {
                    runThread(bottomThread);
                }
                break;
        }



    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current state", liftState);
        lift.telemetry(telemetry);
    }
}
