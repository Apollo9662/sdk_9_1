/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Basic: Omni Linear OpMode test fix", group="Linear OpMode")
@Disabled
public class BasicOmniOpMode_Linear_Test_Fix extends LinearOpMode {

    IMU imu;
    boolean controlMod = false;
    private Motor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private MecanumDrive mecanumDriveBase;
    private GamepadEx driveOp;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();



    // grab the internal DcMotor object

    @Override
    public void runOpMode() {
        driveOp = new GamepadEx(gamepad1);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
        imu.resetYaw();

        frontLeftDrive = new Motor(hardwareMap, "left_front_drive");
        backLeftDrive = new Motor(hardwareMap, "left_back_drive");
        frontRightDrive = new Motor(hardwareMap, "right_front_drive");
        backRightDrive = new Motor(hardwareMap, "right_back_drive");

        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //frontRightDrive.setInverted(false);
        mecanumDriveBase = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            double forwardSpeed   = -driveOp.getLeftY();  // Note: pushing stick forward gives negative value
            double strafeSpeed =  driveOp.getLeftX();
            double turnSpeed     =  driveOp.getRightX();

             */

            double forwardSpeed   = driveOp.getLeftY();  // Note: pushing stick forward gives negative value
            double strafeSpeed =  -driveOp.getLeftX();
            double turnSpeed     =  -driveOp.getRightX();
            double heading  = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (driveOp.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON))
            {
                controlMod = true;
            }
            else
            {
                controlMod = false;
            }
            if (controlMod == true)
            {
                mecanumDriveBase.driveFieldCentric(strafeSpeed/2, forwardSpeed/2, turnSpeed/2, heading);
            }
            else
            {
                mecanumDriveBase.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
            }
            //mecanumDriveBase.driveFieldCentric(strafeSpeed / 2, forwardSpeed / 2, turnSpeed / 2, heading);
            //mecanumDriveBase.driveRobotCentric(strafeSpeed,forwardSpeed,turnSpeed);
        }
    }
}

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
