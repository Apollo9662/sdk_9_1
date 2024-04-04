package org.firstinspires.ftc.teamcode.Unit_Test;
/*
* Copyright (c) 2021 FIRST. All rights reserved.
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



import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoDrive.AutoDriveApollo;
import org.firstinspires.ftc.teamcode.AutoDrive.HuskyLens_Apollo;
import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotHardware_apollo;
import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotHardware_apollo_FtcLib;
import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotMove_apollo;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="UnitTest auto drive", group="Unit Test")
//@Disabled
public class UnitTest_autoDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    public double turnSpeed = 0;
    public double driveSpeed = 0;
    public double targetHeading = 0;
    public double desiredHeading = 90;
    public double headingOffset = 0;
    public double headingError = 0;
    public double robotHeading = 0;
    public final double P_TURN_GAIN = 0.03;     // Larger is more responsive, but also less stable ; PLAY WITH THIS
    public final double P_DRIVE_GAIN = 0.03;
    public final double TURN_SPEED = 0.8 * 0.65;
    private double heading = 0;
    RobotHardware_apollo_FtcLib robot_FtcLib = new RobotHardware_apollo_FtcLib();
    RobotMove_apollo robot = new RobotMove_apollo();
    private GamepadEx gamepadEx1;


    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);
        robot.Robot.init(hardwareMap,false,true);
        robot_FtcLib.init(hardwareMap,false);
        robot.Robot.ServoInit();
        waitForStart();
        while (opModeIsActive())
        {
            gamepadEx1.readButtons();
            double drivingForwardSpeed   = gamepadEx1.getLeftY();  // Note: pushing stick forward gives negative value
            double drivingStrafeSpeed =  gamepadEx1.getLeftX();
            double drivingTurnSpeed     =  -gamepadEx1.getRightX();
            double heading = getRawHeading() - headingOffset;
            //heading = getRawHeading();
            if(gamepadEx1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON))
            {
                desiredHeading = 45;
                holdHeading(TURN_SPEED,desiredHeading);
            }
            else if(gamepadEx1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON))
            {
                desiredHeading = 90;
                holdHeading(TURN_SPEED,desiredHeading);
            }
            else
            {
                robot_FtcLib.driveFieldCentric(drivingStrafeSpeed,drivingForwardSpeed,drivingTurnSpeed,heading);
            }
            if(gamepad1.a == true)
            {
                headingOffset = getRawHeading();
            }
            telemetry.addData("robot heading is ","(%.2f)" + heading);
            telemetry.addData("desired robot heading is ","(%.2f)" + desiredHeading);
            telemetry.update();
        }

    }
    public void holdHeading(double maxTurnSpeed, double heading) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);

        // Stop all motion;
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public double getRawHeading() {
        double angles = robot.Robot.getImuRawHeading();
        //Log.d(TAG_DRIVE, "robot angle. " + angles);
        return angles;
    }
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        double backLeftPower = drive - turn;
        double backRightPower = drive + turn;
        double frontRightPower = drive + turn;
        double frontLeftPower = drive - turn;

        double maxFront = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        double maxBack = Math.max(Math.abs(backLeftPower), Math.abs(backRightPower));
        double max = Math.max(maxFront, maxBack);

        if (max > 1) {
            backLeftPower /= max;
            backRightPower /= max;
            frontLeftPower /= max;
            frontRightPower /= max;
        }
        /*
        Log.d(TAG_DRIVE, "Wheel turn is " + turn);
        Log.d(TAG_DRIVE, "Wheel Speeds is; " +
                " back Left Power is " + backLeftPower +
                " back Right Power is " + backRightPower +
                " front Right Power" + frontRightPower +
                " front Left Power" + frontLeftPower);

         */
        robot_FtcLib.SetPower(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, -backLeftPower);
        robot_FtcLib.SetPower(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightPower);
        robot_FtcLib.SetPower(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightPower);
        robot_FtcLib.SetPower(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, -frontLeftPower);
    }
}

