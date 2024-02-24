package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;

import java.util.concurrent.CompletableFuture;

@TeleOp(name="UnitTest Collection Dump speed Conf", group="Unit Test")
//@Disabled
public class UnitTest_Collection_Dump_conf extends LinearOpMode {
    private DcMotor Collection = null;
    RobotHardware_apollo robot = new RobotHardware_apollo();
    double power = 1;
    double position = 100;
    GamepadEx gamepadEx1;

    @Override
    public void runOpMode()
    {
        hardwareMap.get(DcMotorEx.class, "collection").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.init(hardwareMap,false,false);
        robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.ZeroPowerBehavior.BRAKE);
        gamepadEx1 = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive())
        {
            gamepadEx1.readButtons();
            //power = Range.clip(power,-1000,1000);

            double currentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.COLLECTION);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B))
            {
                position += 1;
            }
            else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X))
            {
                position -= 1;
            }
            if (gamepadEx1.isDown(GamepadKeys.Button.DPAD_UP))
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,power);
            }
            else if (gamepadEx1.isDown(GamepadKeys.Button.DPAD_DOWN))
            {
                robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.COLLECTION, -(int) (currentPosition += position));
                robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.RUN_TO_POSITION);
                robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,power);
            }
            else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT
            ))
            {
                power += 0.05;
            }
            else if (gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            {
                power -= 0.005;
            }
            else
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,0);
            }
            telemetry.addData("position is ", "%1.2f" ,  position);
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

