package org.firstinspires.ftc.teamcode.Unit_Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotHardware_apollo;

@Config
@TeleOp(name="UnitTest Collection Conf", group="Unit Test")
//@Disabled
public class UnitTest_Collection_Conf extends LinearOpMode {
    private DcMotor Collection = null;
    RobotHardware_apollo robot = new RobotHardware_apollo();
    public static double power = 1;
    boolean press;




    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap,false,false);
        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.right_bumper == true)
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION , power);
            }
             else if (gamepad1.left_bumper == true)
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION , -power);
            }
            else
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION , 0);
            }
            if (gamepad1.x == true)
            {
                if (press == false)
                {
                    press = true;
                    if (power < 1.0)
                    {
                        power += 0.1;
                    }
                }
            }
            else if (gamepad1.b == true)
            {
                if (press == false)
                {
                    press = true;
                    if(power >= 0.1)
                    {
                        power -= 0.1;
                    }
                }

            }
            if ((gamepad1.x == false)&& (gamepad1.b == false))
            {
                press = false;
            }

            telemetry.addData("Current Position is " ,"(%.2f)" , robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.COLLECTION));
            telemetry.addData("power is ", "%1.2f" ,  power);
            telemetry.update();
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

