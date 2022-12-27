package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.AS5600;

@Config
public class Turret extends Mechanism{
    public Turret(LinearOpMode opMode) {this.opMode = opMode;}

    private AS5600 as5600;
    private MotionProfiledDcMotor turret1;
    private MotionProfiledDcMotor turret2;

    /*
    5000rpm motor
    1333.3rpm pulley
    169.3 final

    16:60 gear
    16:126 pulley

    100mm diameter / 3.93701 inches

    circumference = 626.11941586 in
    1/4 rotation = 156.529853965 in
     */

    private static final double WHEEL_RADIUS = 3.93701; //inches
    private static final double GEAR_RATIO = 32.0/945.0;
    private static final double TICKS_PER_REV = 28;

    public static double MAX_VEL = 50;
    public static double MAX_ACCEL = 50;
    public static double RETRACTION_MULTIPLIER = 1.0;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    @Override
    public void init(HardwareMap hwMap) {
        turret1 = new MotionProfiledDcMotor(hwMap, "turret1");
        turret1.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        turret1.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        turret1.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        turret1.setPIDCoefficients(kP, kI, kD, kF);

        turret2 = new MotionProfiledDcMotor(hwMap, "turret2");
        turret2.setWheelConstants(WHEEL_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        turret2.setMotionConstraints(MAX_VEL, MAX_ACCEL);
        turret2.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        turret2.setPIDCoefficients(kP, kI, kD, kF);

        // TODO: uncomment as necessary
//        turret1.setDirection(DcMotorSimple.Direction.REVERSE);
        turret2.setDirection(DcMotorSimple.Direction.REVERSE);

        as5600 = new AS5600(hwMap, "as5600", 0, 3.3, -3.3);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("turret1Pos", turret1.getPosition());
        telemetry.addData("turret2Pos", turret2.getPosition());
        telemetry.addData("angle", as5600.getAngle());
    }

    public void update() {
        turret1.update();
        turret2.update();
        as5600.update();
    }

    public void goStraight() {
        if (as5600.getAngle() == 0) {
            turret1.setPower(0);
            turret2.setPower(0);
        } else {

        }
    }

    public void goTurn90() {
        if (as5600.getAngle() == Math.toRadians(90)) {
            turret1.setPower(0);
            turret2.setPower(0);
        } else {
            turret1.setPower(1);
            turret2.setPower(1);
        }
    }

    public void goManual() {
        turret1.setPower(0);
        turret2.setPower(0);
    }

    public void turnTurret(double power) {
        turret1.setPower(power);
        turret2.setPower(power);
    }
}
