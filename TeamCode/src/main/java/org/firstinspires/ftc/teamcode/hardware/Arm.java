package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm extends Mechanism{
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integral = 0;
    public DcMotor motors[] = new DcMotor[2];
    private double angles[] = new double[2];
    public static double tpr = 2786.2;
    public static double tpd = tpr/360;
    public static double kP = -0.0000001;
    public static double kD = 0;
    public static double kI = -0.0000001;
    public static double kCos = 0;
    public static double kBruh = 0;
    public static double targetPos[] = new double[2];
    public static PIDCoefficients coeffs = new PIDCoefficients(kP, kD, kI);
    public static PIDFController controller1 = new PIDFController(coeffs, 0, 0, 0, (angle, angularVel) -> kCos*Math.cos(Math.toRadians(angle)));
    public static PIDFController controller2 = new PIDFController(coeffs, 0, 0, 0, (angle, angularVel) -> kCos*Math.cos(Math.toRadians(angle)));
    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotor.class, "upper");
        motors[1] = hwMap.get(DcMotor.class, "lower");
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        angles[0] = motors[0].getCurrentPosition() / tpd;
        angles[1] = motors[1].getCurrentPosition() / tpd;
    }

    public void setPos(double angle, int motor) {
        double position = angleToPos(angle, motor);
        if(motor == 1) {
            controller1.setTargetPosition(position);
        }
        if(motor == 2) {
            controller2.setTargetPosition(position);
        }
        targetPos[motor-1] = angle;
    }

    public void update(int motor) {
        if(posToAngle(motor) <= 95) {
            double position = motors[motor - 1].getCurrentPosition();
            double error = targetPos[motor - 1] - position;
            double deriv = (error - lastError) / timer.seconds();
            integral = integral + (error * timer.seconds());
//            double power = kP * error + kI * integral + kD * deriv + kCos*Math.cos(Math.toRadians(angleToPos(targetPos[motor - 1], motor)));
            double power = kP * error + kI * integral + kD * deriv;
            motors[motor - 1].setPower(power);
            lastError = error;
            timer.reset();
        }
    }

    public double posToAngle(int motor) {
        double angle = 0;
        return motors[motor-1].getCurrentPosition() / tpd - angles[motor-1];
    }

    public double posToAngle(double pos, int motor) {
        return pos / tpd - angles[motor-1];
    }

    public double angleToPos(double angle, int motor) {
        return angle * tpd + angles[motor-1]*tpd;
    }

    public void power(double speed, int motor) {
        double change = kCos;
        if(motor == 2) {change = kBruh;}
        motors[motor-1].setPower(speed+ change * Math.cos(Math.toRadians(angleToPos(targetPos[motor - 1], motor))));
    }

    public void recalibrate() {
        angles[0] = motors[0].getCurrentPosition() / tpd;
        angles[1] = motors[1].getCurrentPosition() / tpd;
    }
}
