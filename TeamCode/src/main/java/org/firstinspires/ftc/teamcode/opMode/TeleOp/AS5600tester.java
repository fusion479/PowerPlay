package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AS5600;

@TeleOp(name = "Analog Encoder Calibration", group = "TestOpModes")
public class AS5600tester extends LinearOpMode {
    public static double tpr = 28;
    public void runOpMode() throws InterruptedException {
        AS5600 as5600 = new AS5600(hardwareMap, "analogtest", 0, 3.3, -3.3);
        DcMotorEx rot = hardwareMap.get(DcMotorEx.class, "turret");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry tele = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        waitForStart();
        ElapsedTime time = new ElapsedTime();
        while(opModeIsActive()) {
            double quad = rot.getCurrentPosition() / tpr * Math.PI * 2;
            double analog = as5600.getAngle();
            double voltage = as5600.getVoltage();
            tele.addData("angle rn: ", quad);
            tele.addData("voltage from as6500: ", voltage);
            tele.addData("angle reading(sus): ", analog);
            tele.update();
        }
    }
}