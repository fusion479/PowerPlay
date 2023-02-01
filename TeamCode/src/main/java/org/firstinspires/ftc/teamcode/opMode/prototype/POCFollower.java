package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class POCFollower extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }
}

class Localizer extends Mechanism {
    //motors will act as our encoders
    DcMotor left;
    DcMotor right;
    DcMotor front;
    double LEFT = 1;
    double RIGHT = 2;
    double FRONT = 3;

    //encoder stats
    final double TICKS_PER_REV = 8192; //through bore tpr
    final double WHEEL_RADIUS = 0.6889764; //radius of a 35mm rotacaster wheel
    final double TICKS_PER_RADIAN = 8192 / (2*Math.PI);

    //odometry stats
    static double x_0 = 10; //location of front encoder
    static double y_0 = 10; //location of side encoders relative to center


    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.get(DcMotor.class, "left");
        right = hwMap.get(DcMotor.class, "right");
        front = hwMap.get(DcMotor.class, "front");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getAngularDisplacement(int encoder) {
        double del_theta = 0;
        if(encoder == LEFT) {
            del_theta = left.getCurrentPosition() / TICKS_PER_RADIAN;
        }
        if(encoder == RIGHT) {
            del_theta = right.getCurrentPosition() / TICKS_PER_RADIAN;
        }
        if(encoder == FRONT) {
            del_theta = front.getCurrentPosition() / TICKS_PER_RADIAN;
        }
        return del_theta;
    }




}
