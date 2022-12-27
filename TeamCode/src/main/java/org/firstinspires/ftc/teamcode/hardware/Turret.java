package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
public class Turret extends Mechanism{
    DcMotorEx turr;
    ElapsedTime timer = new ElapsedTime();
    //CONSTANTS
    public static double kP = 0;
    public static double kD = 0;
    public static double kS = 0;
    public static double vMax = 1;
    public static double tpd = 28 * 250 / 360; //encoder res * gear ratio / 360 degrees = ticks per degree

    public static double target = 0;
    public double lastError = 0; //separate error for each motor

    @Override
    public void init(HardwareMap hwMap) {
        turr = hwMap.get(DcMotorEx.class, "turr");
        turr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turr.setDirection(DcMotorSimple.Direction.FORWARD);
        timer.reset();
    }

    public void update() {
        double time = timer.milliseconds();
        double error = turr.getCurrentPosition() - target;
        double pd = kP * error + kD * (error-lastError) / time;
        lastError = error;
        timer.reset();
        turr.setPower(Range.clip(pd + kS, -vMax, vMax));
    }

    public void setTargetPosition(double pos) {
        target = pos;
    }

    public void setTargetAngle(double angle){ //ANGLE MUST BE IN DEGREES
        target = angle * tpd;
    }

    public double getPos() {
        return turr.getCurrentPosition();
    }

    public double getAngle() {
        return (getPos())/(2*tpd); //motor pos / ticks per degree
    }

    public void setPower(double power) {
        turr.setPower(power);
    }


}
