package org.firstinspires.ftc.teamcode.hardware;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Lift extends Mechanism{
    DcMotorEx motors[] = new DcMotorEx[2];
    TouchSensor limit;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime resetter = new ElapsedTime();
    //CONSTANTS
    public static double kG = 0.0005;
    public static double kP = 0;
    public static double kD = 0;
    public static double bound = 0.02;
    public static double vMax = 1;

    public static double target = 0;
    public double lastError[] = {0, 0}; //separate error for each motor
    public double powers[] = {0,0};
    public boolean isReset = true;
    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "left");
        motors[1] = hwMap.get(DcMotorEx.class, "right");
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        limit = hwMap.get(TouchSensor.class, "limit");
        resetter.reset();
        timer.reset();
    }

    public void update(int motor) {
        double time = timer.milliseconds();
        double error = motors[0].getCurrentPosition() - target;
        double pd = kP * error + kD * (error-lastError[motor]) / time;
        if(Math.abs(error) < bound) {
            pd = 0;
        }
        lastError[motor] = error;
        timer.reset();
        powers[motor] = Range.clip(pd + kG, -vMax, vMax);
        motors[motor].setPower(powers[motor]);
    }

    public void setTargetPosition(double pos) {
        target = pos;
    }

    public void loop() {
        if(limit.isPressed() && !isReset) {
            recalibrate();
        }else {
            update(0);
            update(1);
        }
        isReset = limit.isPressed();
    }
   //TODO: ask kellen how to convert
    public double inchToPos(double inches) {
        return inches; //lol idk how to convert
    }

    public double getPos(int motor) {
        return motors[motor].getCurrentPosition();
    }

    public double getPos() {
        return (getPos(0) + getPos(1)) / 2;
    }

    public void recalibrate() {
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = 0;
        motors[0].setPower(0);
        motors[1].setPower(0);
        resetter.reset();
    }

    public void setPower(double power) {
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    public double getAvgError() {
        return ((target-getPos(0)) + (target-getPos(1))) / 2;
    }

    public double getError(int motor) {
        return target - getPos(motor);
    }

    public double getCurrent(int motor) {
        return motors[motor].getCurrent(CurrentUnit.AMPS);
    }

}
