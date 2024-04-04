/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.RobotHardware_apollo;

import android.util.Log;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware_apollo_FtcLib {

    IMU imu;
    private Motor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private MecanumDrive mecanumDriveBase;
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    final String TAG_HARDWARE = "HardwareApollo";
    // Define a constructor that allows the OpMode to pass a reference to itself.
    //public RobotHardware_apollo(LinearOpMode opmode) {
        //myOpMode = opmode;
    //}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public boolean init(HardwareMap apolloHardwareMap, boolean initImu)
    {
        boolean imuInitialize = true;
        if (initImu)
        {
            imu = apolloHardwareMap.get(IMU.class, "imu2");
            imuInitialize = imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )));
            if (!imuInitialize)
            {
                imu = apolloHardwareMap.get(IMU.class, "imu1");
                imuInitialize = imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )));
            }
            imu.resetYaw();
        }


        frontLeftDrive = new Motor(apolloHardwareMap, "front_left_drive");
        backLeftDrive = new Motor(apolloHardwareMap, "back_left_drive");
        frontRightDrive = new Motor(apolloHardwareMap, "front_right_drive");
        backRightDrive = new Motor(apolloHardwareMap, "back_right_drive");

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //frontRightDrive.setInverted(true);
        /*
        backRightDrive.setInverted(true);
        frontLeftDrive.setInverted(true);
        backLeftDrive.setInverted(true);

         */
        mecanumDriveBase = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        return (imuInitialize);

    }
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading)
    {
        mecanumDriveBase.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed)
    {
        mecanumDriveBase.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }
    public double getYaw()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void ResetYaw()
    {
        imu.resetYaw();
    }
    public void SetAllDriveMotorsMode(Motor.RunMode myMode)
    {
        frontLeftDrive.setRunMode(myMode);
        frontRightDrive.setRunMode(myMode);
        backRightDrive.setRunMode(myMode);
        backLeftDrive.setRunMode(myMode);
    }
    public void SetAllMotorsZeroPowerBehavior(Motor.ZeroPowerBehavior myZeroPowerBehavior)
    {
        frontLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        frontRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        backRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        backLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
    }
    public void SetPower(RobotHardware_apollo.DriveMotors motor, double Power)
    {
        switch (motor)
        {

            case BACK_LEFT_DRIVE:
            {
                backLeftDrive.set(Power);
            }
            break;
            case BACK_RIGHT_DRIVE:
            {
                backRightDrive.set(Power);
            }
            break;
            case FRONT_LEFT_DRIVE:
            {
                frontLeftDrive.set(Power);
            }
            break;
            case FRONT_RIGHT_DRIVE:
            {
                frontRightDrive.set(Power);
            }
            break;
            default:
                break;
        }
    }


}


