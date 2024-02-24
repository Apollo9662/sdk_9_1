package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware_apollo;

@Config
@TeleOp (name="PID Collection dash borad", group="Unit Test")
@Disabled
public class PID_Collection_dashBoard extends OpMode {
    RobotHardware_apollo robot = new RobotHardware_apollo();
    private PIDController controller;
    public static double Kp = 0, Ki = 0, Kd = 0;
    public static double Kf = 0;
    public static int target = 0;
    double highScore = 0;
    double score = 0;
    public static int offset = 50;
    double velocity;
    //1private final double ticks_in_degree = 537.6 / 360;
    @Override
    public void init(){

        controller = new PIDController(Kp, Ki, Kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, false,false);

    }

    @Override
    public void loop()
    {
        //velocity = robot.GetVelocity(RobotHardware_apollo.DriveMotors.COLLECTION);

        controller.setPID(Kp,Ki,Kd);
        velocity = robot.GetVelocity(RobotHardware_apollo.DriveMotors.COLLECTION);
        double pid = controller.calculate(velocity,target);
        double ff = target * Kf;
        double power = pid + ff;
        power = Range.clip(power,-1,1);
        //robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,power);
        robot.SetVelocity(RobotHardware_apollo.DriveMotors.COLLECTION, target);
        telemetry.addData("velocity" , velocity);
        telemetry.addData("target", target);
        telemetry.addData("highScore" , highScore);
        telemetry.update();
    }
}
