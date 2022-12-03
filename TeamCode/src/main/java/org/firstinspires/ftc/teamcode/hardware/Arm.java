package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class Arm extends Mechanism{
    private DcMotor motors[] = new DcMotor[2];

    private double tpr = 751.8;
    static public double kP = 1;
    static public double kD = 0;
    static public double kI = 0;
    static public double kCos = 0;
    public static PIDCoefficients coeffs = new PIDCoefficients(kP, kD, kI);
    public static PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (angle, angularVel) -> kCos*Math.cos(angle));

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotor.class, "arm1");
        motors[1] = hwMap.get(DcMotor.class, "arm2");
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPos(double angle, int motor) {
        double position = angleToPos(angle);
        controller.setTargetPosition(position);
        motors[motor-1].setPower(controller.update(motors[motor-1].getCurrentPosition()));

    }

    public void update() {

    }

    public double posToAngle(int motor) {
        double angle = 0;
        return motors[motor-1].getCurrentPosition() / tpr * 360;
    }

    public double angleToPos(double angle) {
        return angle/360 * tpr;
    }

    public void power(double speed, int motor) {
        motors[motor-1].setPower(speed);
    }

    public double getAngle(int motor) {
        return angleToPos(motors[motor-1].getCurrentPosition());
    }

}
