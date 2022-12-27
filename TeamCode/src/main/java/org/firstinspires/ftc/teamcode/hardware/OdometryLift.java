package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryLift extends Mechanism {
    public OdometryLift(LinearOpMode opMode) {this.opMode = opMode;}

    private Servo odoLift;

    public static double UP_POS = .5;
    public static double DOWN_POS = 0;

    public enum OdoLiftState {
        UP,
        DOWN,
    };

    public OdoLiftState odoLiftState;

    @Override
    public void init(HardwareMap hwMap) {
        odoLift = hwMap.get(Servo.class, "odoLift");

        down();
    }

    public void loop(Gamepad gamepad) {
        switch (odoLiftState) {
            case UP:
                if (gamepad.dpad_left) {
                    down();
                }
            case DOWN:
                if (gamepad.dpad_left) {
                    up();
                }
        }
    }

    public void up() {
        odoLift.setPosition(UP_POS);
    }

    public void down() {
        odoLift.setPosition(DOWN_POS);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("odoLift", odoLiftState);
    }
}
