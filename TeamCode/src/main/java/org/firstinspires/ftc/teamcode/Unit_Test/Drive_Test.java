package org.firstinspires.ftc.teamcode.Unit_Test;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name="drive Test", group="Unit Test")
public class Drive_Test extends OpMode {

    private Motor fL, fR, bL, bR;
    public static boolean resetYaw = false;
    private BNO055IMU imu = null;
    boolean imuInitialize = true;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    double headingOffset = 0;

    @Override
    public void init() {
        /* instantiate motors */
        fL = new Motor(hardwareMap, "front_left_drive");
        bL = new Motor(hardwareMap, "back_left_drive");
        fR = new Motor(hardwareMap, "front_right_drive");
        bR = new Motor(hardwareMap, "back_right_drive");
        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuInitialize = imu.initialize(parameters);
        if (imuInitialize == false)
        {
            imu = hardwareMap.get(BNO055IMU.class, "imu1");
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuInitialize = imu.initialize(parameters);
        }
        drive = new MecanumDrive(fL, fR, bL, bR);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (resetYaw)
        {
            resetHeading();
        }
        double heading = getImuRawHeading();
        heading -= headingOffset;
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                -driverOp.getRightX(),
                heading

        );
        telemetry.addData("heading" ,"(%.2f)", heading);
        telemetry.update();
    }
    public double getImuRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getImuRawHeading();
    }

}
