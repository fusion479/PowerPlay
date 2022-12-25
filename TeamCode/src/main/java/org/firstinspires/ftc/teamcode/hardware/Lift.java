package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift extends Mechanism {
    private MotionProfiledDcMotor leftLiftMotor;
    private MotionProfiledDcMotor rightLiftMotor;

    private TouchSensor leftTouchSensor;
    private TouchSensor rightTouchSensor;

    //TODO: change these to match motor
    private static final double WHEEL_RADIUS = .797; //inches
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 145.1;

    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;
    public static double RETRACTION_MULTIPLIER = 0.7;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static int POS_LOW = 15;
    public static int POS_MEDIUM = 25;
    public static int POS_HIGH = 35;

    public Lift(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftLiftMotor = new MotionProfiledDcMotor(hwMap, "leftLiftMotor");
        leftLiftMotor.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        leftLiftMotor.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        leftLiftMotor.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        leftLiftMotor.setPIDCoefficients(kP, kI, kD, kF);

        rightLiftMotor = new MotionProfiledDcMotor(hwMap, "rightLiftMotor");
        rightLiftMotor.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        rightLiftMotor.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        rightLiftMotor.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        rightLiftMotor.setPIDCoefficients(kP, kI, kD, kF);

        // TODO: reverse as necessary
        // leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftTouchSensor = hwMap.touchSensor.get("leftTouchSensor");
        rightTouchSensor = hwMap.touchSensor.get("rightTouchSensor");

        goBottom();
    }

    public void goBottom() {
        leftLiftMotor.setTargetPosition(0);
        rightLiftMotor.setTargetPosition(0);
    }

    public void goLow() {
        leftLiftMotor.setTargetPosition(POS_LOW);
        rightLiftMotor.setTargetPosition(POS_LOW);
    }

    public void goMedium() {
        leftLiftMotor.setTargetPosition(POS_MEDIUM);
        rightLiftMotor.setTargetPosition(POS_MEDIUM);
    }
    public void goHigh() {
        leftLiftMotor.setTargetPosition(POS_HIGH);
        rightLiftMotor.setTargetPosition(POS_HIGH);
    }

    public void update() {
        leftLiftMotor.update();
        rightLiftMotor.update();
    }

    // TODO: implement ?
    public boolean touchSensor() {
        return leftTouchSensor.isPressed() && rightTouchSensor.isPressed();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("leftSlidePos", leftLiftMotor.getPosition());
        telemetry.addData("rightSlidePos", rightLiftMotor.getPosition());
        telemetry.addData("leftTouch", leftTouchSensor.isPressed());
        telemetry.addData("rightTouch", rightTouchSensor.isPressed());
    }
}
