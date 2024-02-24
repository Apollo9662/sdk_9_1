package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware_apollo;

import java.util.Currency;

@Config
@TeleOp(name="PID Collection", group="Unit Test")
@Disabled
public class PID_Collection extends OpMode {
    RobotHardware_apollo robot = new RobotHardware_apollo();
    double integralSum = 0;
    double lestError = 0;
    public static double Kp = 0, Ki = 0, Kd = 0;
    public static double Kf = 0;
    double velocity = 0;
    double position = 0;
    double power = 0;
    double max = 1;
    double min = 0.6;
    double lestPower = 0;
    double highScore = 0;
    double score = 0;
    int offset = 50;
    int target = 1000;
    final String TAG_COEFFICIENT = "coefficients";
    ElapsedTime time = new ElapsedTime();

    public double PIDCollectionControl(double reference, double state)
    {
        double error = reference - state;
        Log.d(TAG_COEFFICIENT, "error is " + error);
        //Log.d(TAG_COEFFICIENT, "time is " + time.seconds());
        integralSum += error * time.seconds();
        double derivative = (error - lestError) / time.seconds();
        lestError = error;
        time.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        lestPower = output;
        return output;
    }

    @Override
    public void init() {
        robot.init(hardwareMap,false,false);
        PIDFCoefficients pidOrig = robot.GetPIDFCoefficients(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("init");
        telemetry.addData("PIDF Coefficients are P,I,D,F " , "%.5f, %.5f, %.5f, %.5f" ,pidOrig.p, pidOrig.i , pidOrig.d, pidOrig.f);
        telemetry.update();
    }

    @Override
    public void loop() {

            velocity = robot.GetVelocity(RobotHardware_apollo.DriveMotors.COLLECTION);
            position = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.COLLECTION);

            power = PIDCollectionControl(-target,velocity);
            robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, power);

            if ((velocity > (target - offset)) && (velocity < (target + offset)))
            {
                score += 1;
            }
            else
            {
                if (score >= highScore)
                {
                    highScore = score;
                }
                score = 0;
            }

            if ((gamepad1.y) || (gamepad1.a))
            {
                Log.d(TAG_COEFFICIENT,"high score is " + highScore);
                Log.d(TAG_COEFFICIENT, "last power is " + lestPower);
                Log.d(TAG_COEFFICIENT, "power var is " + power);
                //PIDCoefficients pidOrig = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                Log.d(TAG_COEFFICIENT, "velocity is " + velocity);
                //Log.d(TAG_COEFFICIENT, "PIDF Coefficients are P,I,D,F " + pidOrig.p + pidOrig.i + pidOrig.d + pidOrig.f);
                //Log.d(TAG_COEFFICIENT,"Current Position is: " + position);
                //Log.d(TAG_COEFFICIENT,"velocity is " + robot.GetVelocity(RobotHardware_apollo.DriveMotors.COLLECTION));
                Log.d(TAG_COEFFICIENT, "power is " + robot.GetPower(RobotHardware_apollo.DriveMotors.COLLECTION));
            }
            //telemetry.addData("Current Position is " ,"(%.2f)" , position);
            telemetry.addData("velocity is " ,"(%.2f)" , velocity);
            telemetry.addData("power is " ,"(%.2f)" , power);
            telemetry.update();
    }
}
