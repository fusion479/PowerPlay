package org.firstinspires.ftc.teamcode.hardware;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
public class Lift extends Mechanism{
    DcMotorEx motors[] = new DcMotorEx[2];
    TouchSensor limits[] = new TouchSensor[2];
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime resetter = new ElapsedTime();
    //CONSTANTS
    public static double kG = 0;
    public static double kP = 0;
    public static double kD = 0;
    public static double bound = 0.02;
    public static double vMax = 1;

    public static double target = 0;
    public double lastError[] = new double[2]; //separate error for each motor
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
        limits[0] = hwMap.get(TouchSensor.class, "limitL");
        limits[1] = hwMap.get(TouchSensor.class, "limitR");
        resetter.reset();
        timer.reset();
    }

    public void update(int motor) {
        double time = timer.milliseconds();
        double error = motors[motor].getCurrentPosition() - target;
        double pd = kP * error + kD * (error-lastError[motor]) / time;
        if(Math.abs(error) < bound) {
            pd = 0;
        }
        lastError[motor] = error;
        timer.reset();
        motors[motor].setPower(Range.clip(pd + kG, -vMax, vMax));
    }

    public void setTargetPosition(double pos) {
        target = pos;
    }

    public void loop() {
        if(!(limits[1].isPressed() || limits[0].isPressed())) {
            update(0);
            update(1);
            //if(resetter.milliseconds() > 1) { //might need this in case lift doesn't move off the switch before the conditional catches itself
                isReset = false;
            //}
        }else if(!isReset){
            recalibrate();
            target = 0;
            motors[0].setPower(0);
            motors[1].setPower(0);
            isReset = true;
        }
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
        resetter.reset();
    }

    public void setPower(double power) {
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

}
