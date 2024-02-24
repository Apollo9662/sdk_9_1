package org.firstinspires.ftc.teamcode;
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



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="UnitTest plane servo Conf", group="Unit Test")
//@Disabled
public class UnitTest_PlaneServo_Conf extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    RobotHardware_apollo robot = new RobotHardware_apollo();
    boolean press = false;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false, false);
        waitForStart();
        while (opModeIsActive())
        {

            if(gamepad1.a == true)
            {
                if (press == false)
                {
                    press = true;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO, robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO) + 0.01);

                }
            }
            else if(gamepad1.y == true)
            {
                if (press == false)
                {
                    press = true;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO, robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO) - 0.01);

                }
            }
            /*
            if(gamepad1.x == true)
            {
                if (press == false)
                {
                    press = true;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.COLLECTION_SERVO, 1);


                }
                }
            else if(gamepad1.b == true)
            {
                if (press == false)
                {
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.COLLECTION_SERVO, 0);
                    press = true;
                }

            }
            /*
             */
            if (gamepad1.a == false && gamepad1.b == false && gamepad1.x == false && gamepad1.y == false)
            {
                press = false;
            }
            telemetry.addData("servo Position is ", "(%.2f)", robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO));
            telemetry.update();
        }
    }
}

