package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Claw extends Mechanism {
    public Claw(LinearOpMode opMode) {this.opMode = opMode;}

    private Servo claw;

    public static double OPEN_POS = 0;
    public static double CLOSED_POS = 1;

    public enum ClawState {
        OPEN,
        CLOSED,
    };

    public ClawState clawState;

    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        // TODO: uncomment as necessary
//        claw.setDirection(Servo.Direction.REVERSE);
    }

    public void loop(Gamepad gamepad) {
        switch (clawState) {
            case OPEN:
                if (gamepad.left_bumper) {
                    open();
                }
                break;
            case CLOSED:
                if (gamepad.left_bumper) {
                    close();
                }
        }

    }

    public void open() {
        claw.setPosition(OPEN_POS);
    }

    public void close() {
        claw.setPosition(CLOSED_POS);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("clawPos", claw.getPosition());
    }
}
