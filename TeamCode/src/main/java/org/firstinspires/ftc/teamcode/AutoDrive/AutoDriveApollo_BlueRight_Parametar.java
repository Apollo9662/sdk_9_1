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

package org.firstinspires.ftc.teamcode.AutoDrive;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotHardware_apollo;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="blue right", group="Apollo")
//@Disabled
public class AutoDriveApollo_BlueRight_Parametar extends LinearOpMode {

    /* Declare OpMode members. */


    HuskyLens_Apollo.PropPos detectedPropPos = null;
    AutoDriveApollo autoDriveApollo = new AutoDriveApollo(this);
    //enum ProbPos{UP,
    //RIGHT,
    //LEFT}
    //ProbPos probPos;
    //private DcMotor         frontLeftDrive   = null;
    //private DcMotor         frontRightDrive  = null;
    //private DcMotor         backLeftDrive    = null;
    //private DcMotor         backRightDrive   = null;
    //private Servo armServo;
    //private Servo armGardServo;
    //public DcMotor lift = null;
    //private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.

    //RobotHardware_apollo robot = new RobotHardware_apollo();
    //HuskyLens_Apollo robotHuskLens = new HuskyLens_Apollo();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        autoDriveApollo.init(HuskyLens_Apollo.PropColor.BLUE);
        telemetry.addLine("robot finish init");

        //Log
        telemetry.addLine("failed the detect Prop");
        //sleep(1000);
        telemetry.update();

        //detectedPropPos = autoDriveApollo.runPropDetection();
        detectedPropPos = HuskyLens_Apollo.PropPos.UP;
        waitForStart();
        autoDriveApollo.liftTread.start();
        runAutoDrive_BlueRight();
        //}
        //driveStraight(DRIVE_SPEED, 23 * 2, 0 );


        /*
        driveStraight(DRIVE_SPEED,-23 * 4,0);
        turnToHeadingApollo(TURN_SPEED,-90);
        driveStraight(DRIVE_SPEED,-23,-90);
        turnToHeadingApollo(TURN_SPEED,180);
        driveStraight(DRIVE_SPEED,-20 * 4,180);
        turnToHeadingApollo(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-20,90);
        turnToHeadingApollo(TURN_SPEED,0);
        driveStraight(DRIVE_SPEED/2,7,0);

         */


        //driveLeft(DRIVE_SPEED, 10 ,0);
        //driveStraight(DRIVE_SPEED, 20.5, 0.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
        //turnToHeading( TURN_SPEED,   -90.0);               // Turn  CW  to 0 Degrees


        sleep(1000);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        autoDriveApollo.liftTread.interrupt();
        //sleep(1000);  // Pause to display last telemetry message.
    }

    public void Park_BlueRight(int heading, HuskyLens_Apollo.PropPos propPos, boolean park) {
        if (park) {
            switch (propPos) {
                case UP: {
                    autoDriveApollo.robot.MoveServo.openGard();
                    //autoDriveApollo.robot.SetPower.collection(0.1);
                    //autoDriveApollo.robot.SetPower.collection(1);
                    //autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -2, heading);
                    //sleep(250);
                    autoDriveApollo.robot.SetPower.collection(-1);
                    double headingForCollecting = heading;
                    while ((autoDriveApollo.pixelInCollection == 0) && opModeIsActive())
                    {
                        autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED,headingForCollecting);
                        headingForCollecting += 1;
                        sleep(1);
                    }
                    Log.d(autoDriveApollo.TAG_Distance,"collected pixel and im wating");
                    Log.d(autoDriveApollo.TAG_Distance,"num of pixels is " + autoDriveApollo.pixelInCollection);
                    while ((autoDriveApollo.duringCollection) && (opModeIsActive()) && autoDriveApollo.pixelInCollection == 1)
                    {
                        sleep(1);
                    }
                    Log.d(autoDriveApollo.TAG_Distance,"num of pixels is " + autoDriveApollo.pixelInCollection);
                    autoDriveApollo.robot.SetPower.collection(1);
                    //sleep(100);
                    autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 6, heading);
                    autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,1);
                    autoDriveApollo.robot.SetPower.collection(0);
                    autoDriveApollo.driveRight(autoDriveApollo.DRIVE_SPEED, 17, heading);
                    autoDriveApollo.robot.SetPower.collection(1);
                    //autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-2,heading);
                    //autoDriveApollo.robot.SetPower.collection(0.3);
                }
                break;
                case LEFT: {
                    autoDriveApollo.driveRight(autoDriveApollo.DRIVE_SPEED, 26, heading);
                }
                break;
                case RIGHT: {
                    autoDriveApollo.driveRight(autoDriveApollo.DRIVE_SPEED, 42, heading);
                    autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 4, heading);
                }
                break;
            }
            //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
            if (autoDriveApollo.DropPixelAtBack) {
                if (autoDriveApollo.collectFirstPixel) {
                    //autoDriveApollo.robot.SetPower.collection(0.3);
                    /*
                    autoDriveApollo.robot.MoveServo.openGard();
                    autoDriveApollo.robot.SetPower.collection(0.4);
                    autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -6, heading);
                    autoDriveApollo.robot.SetPower.collection(-1);
                    autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 23 * 2, heading);
                    autoDriveApollo.robot.SetPower.collection(1);
                    autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 23, heading);
                    //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);

                     */
                    autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 23 * 3, heading);
                    autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED, heading, 0.5);
                } else {
                    autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 23 * 3, heading);
                    autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED, heading, 0.5);
                }
            } else {
                while (autoDriveApollo.runTime.seconds() <= autoDriveApollo.TimeToPark && ((opModeIsActive()))) {
                    Log.d(autoDriveApollo.TAG_DRIVE, "whiting to park and drop pixel");
                }
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 23 * 4, heading);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED, heading, 0.5);
                //autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED,2,heading);
            }

            //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
            //autoDriveApollo.driveStraight(autoDriveApollo.TURN_SPEED,15,heading);
        }

    }

    public void dropPixelAtLine_BlueRight(double heading, HuskyLens_Apollo.PropPos probPos) {
        switch (probPos) {
            case UP: {
                heading = 0;
                /*
                autoDriveApollo.driveRight(autoDriveApollo.DRIVE_SPEED,6,heading);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,5,heading);
                //driveLeft(DRIVE_SPEED,3,heading);

                 */
                autoDriveApollo.robot.Robot.SetPosition(RobotHardware_apollo.DriveMotors.DUMP_SERVO, RobotHardware_apollo.SERVO_POS.DUMP_UNLOAD_PIXEL.Pos);
                sleep(500);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 7, heading);
                //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,1);
                autoDriveApollo.robot.Robot.SetPosition(RobotHardware_apollo.DriveMotors.DUMP_SERVO, RobotHardware_apollo.SERVO_POS.DUMP_LOAD_PIXEL.Pos);
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED, -90);
            }
            break;
            case LEFT: {
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED, -90);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 6, -90);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED, -90, 0.5);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -5, -90);
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED, 0);
                //autoDriveApollo.robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,0);
                heading = 0;
                autoDriveApollo.driveRight(autoDriveApollo.DRIVE_SPEED, 2.5, heading);
                autoDriveApollo.releasePixel(heading, false);
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED, -90);

            }
            break;
            case RIGHT: {
                //driveStraight(DRIVE_SPEED,10,heading);
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED, -90);
                heading = -90;
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 2, heading);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 4, 0);
                //autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED,3,heading);
                autoDriveApollo.releasePixel(heading, false);
                //autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED,-90);
                //driveRight(DRIVE_SPEED,10,heading);
                //holdHeading(TURN_SPEED,heading,1);
            }
            break;
        }

    }

    public void driveToBackStage_BlueRight(double heading, HuskyLens_Apollo.PropPos probPos) {
        switch (probPos) {
            case RIGHT: {
                //driveRight(DRIVE_SPEED, 7, heading);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 18, heading);
            }
            break;
            case LEFT: {
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 28, heading);
            }
            break;
            case UP: {
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 20, heading);
            }
            break;
        }
        autoDriveApollo.robot.SetPower.collection(0);
        autoDriveApollo.robot.MoveServo.closeGard();
        autoDriveApollo.getLiftToFarDumpPos();
        //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
        switch (probPos) {
            case RIGHT: {
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 7, heading);
            }
            break;
            case UP:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 10, heading);
                break;
            case LEFT:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 5, heading);
                break;
        }
        //autoDriveApollo.holdHeading(autoDriveApollo.DRIVE_SPEED, heading, 0.5);
        switch (probPos) {
            case RIGHT: {
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED - 0.2, 4, heading);
            }
            break;
            case UP: {
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED - 0.2, 4, heading);
            }
            break;
        }
        autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED - 0.2, 4, heading);
        //sleep(1000);
        autoDriveApollo.robot.Robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos);
        sleep(200);
        autoDriveApollo.robot.Robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos);
        sleep(200);
        //sleep(1000);
        /*
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_USING_ENCODER);
        int Pos = (int) (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
        robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT, Pos + 100);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, 1);
        TimeOut.reset();
        LIFT_IsBusy = robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
        while ((LIFT_IsBusy) && (TimeOut.seconds() > TimeOutSec))
        {
            LIFT_IsBusy = robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
        }

        /*
        ElapsedTime liftTime = new ElapsedTime();
        liftTime.reset();
        boolean isBusy = robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
        while((isBusy == true) && (liftTime.seconds() < 3)) {
            isBusy = robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
        }

         */

    }

    public void driveToProb_BlueRight(HuskyLens_Apollo.PropPos propPos)
    {
        int heading = 0;
        autoDriveApollo.MoterTime.reset();
        if (propPos == HuskyLens_Apollo.PropPos.RIGHT)
        {
            autoDriveApollo.robot.Robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,0.5);
        }
        else if (propPos == HuskyLens_Apollo.PropPos.UP)
        {// 0.2
            autoDriveApollo.robot.Robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,1);
        }
        switch (propPos)
        {
            case UP:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-22,0);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,0,0.5);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED,10,0);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,0,0.5);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-8,0);
                heading = 0;
                break;
            case RIGHT:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-15,0);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,0,0.5);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED,3,0);
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED,-45);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-10,-45);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,-45,0.5);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,2,-45);
                heading = -45;
                /*
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-7,0);

                 */
                //autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED,10,0);

                break;
            case LEFT:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-21,0);
                heading = 0;

                break;
        }
        autoDriveApollo.robot.Robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,0);
        autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
    }
    /*
    public void driveToProb_BlueRight(HuskyLens_Apollo.PropPos propPos) {
        int heading = 0;
        autoDriveApollo.MoterTime.reset();
        if (propPos == HuskyLens_Apollo.PropPos.RIGHT) {
            autoDriveApollo.robot.Robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, 0.5);
        } else if (propPos == HuskyLens_Apollo.PropPos.UP) {
            autoDriveApollo.robot.Robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, 0.2);
        }
        switch (propPos) {
            case UP:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -22, 0);
                //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,0,0.5);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 10, 0);
                //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,0,0.5);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -5, 0);
                heading = 0;
                break;
            case RIGHT:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -15, 0);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED, 0, 0.5);
                autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED, 3, 0);
                autoDriveApollo.turnToHeadingApollo(autoDriveApollo.TURN_SPEED, -45);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -10, -45);
                autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED, -45, 0.5);
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 2, -45);
                heading = -45;
                /*
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED,-7,0);


                //autoDriveApollo.driveLeft(autoDriveApollo.DRIVE_SPEED,10,0);

                break;
            case LEFT:
                autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -21, 0);
                heading = 0;

                break;
        }
        autoDriveApollo.robot.Robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, 0);
        //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
    }

     */
    public void collectSecondPixel(int heading) {
        if (autoDriveApollo.collectSecondPixel) {
            autoDriveApollo.driveRight(autoDriveApollo.DRIVE_SPEED, 20, heading);
            //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
            autoDriveApollo.robot.SetPower.collection(1);
            autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -70, heading);
            autoDriveApollo.robot.SetPower.collection(0);
            autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, -15, heading);
            //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
            autoDriveApollo.robot.MoveServo.openGard();
            autoDriveApollo.robot.SetPower.collection(-1);
            autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.1);
            autoDriveApollo.driveStraight(autoDriveApollo.DRIVE_SPEED, 72, heading);
            autoDriveApollo.robot.SetPower.collection(1);
            //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,heading,0.5);
        }

    }

    public void runAutoDrive_BlueRight() {
        autoDriveApollo.time.reset();
        //detectedPropPos = autoDriveApollo.detectProp();
        //autoDriveApollo.robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, 1);
        autoDriveApollo.dropCollection();
        driveToProb_BlueRight(detectedPropPos);
        dropPixelAtLine_BlueRight(0, detectedPropPos);
        Park_BlueRight(-90,detectedPropPos, autoDriveApollo.Park);
        if (autoDriveApollo.DropPixelAtBack)
        {
            //autoDriveApollo.holdHeading(autoDriveApollo.TURN_SPEED,-90,1);
            driveToBackStage_BlueRight(-90, detectedPropPos);
            //sleep(500);
            autoDriveApollo.getReadyForTeleOp(-90,true);
            collectSecondPixel(-90);
            if (autoDriveApollo.collectSecondPixel)
            {
                driveToBackStage_BlueRight(-90,detectedPropPos);
                autoDriveApollo.getReadyForTeleOp(-90,false);
            }
            Log.d(autoDriveApollo.TAG_TIME, "the final time is " + autoDriveApollo.time.milliseconds());
        }
        }

        /*
         * ====================================================================================================
         * Driving "Helper" functions are below this line.
         * These provide the high and low level methods that handle driving straight and turning.
         * ====================================================================================================
         */
    }

