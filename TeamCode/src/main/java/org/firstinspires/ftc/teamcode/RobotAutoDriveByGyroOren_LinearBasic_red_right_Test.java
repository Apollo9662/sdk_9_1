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

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name="Apollo Autonomous red right Basic Test", group="Apollo")
@Disabled
public class RobotAutoDriveByGyroOren_LinearBasic_red_right_Test extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime TimeOut = new ElapsedTime();
    double propDetectionTimeOut = 5;
    final String TAG_TIME = "time";
    final String TAG_DRIVE = "drive";

    HuskyLens_Apollo.PropPos detectedPropPos = null;
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

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     frontLeftTarget    = 0;
    private int     frontRightTarget   = 0;
    private int     backLeftTarget    = 0;
    private int     backRightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.7795275591;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.8;
    static final double     TURN_SPEED_FIX          = 0.4; // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 0.5;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
                                                                //PLAY
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.01;     // Larger is more responsive, but also less stable ; PLAY WITH THIS
    static final double     P_DRIVE_GAIN           = 0.0001;     // Larger is more responsive, but also less stable

    RobotHardware_apollo robot = new RobotHardware_apollo();
    HuskyLens_Apollo robotHuskLens = new HuskyLens_Apollo();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap,true,true);
        boolean initHuskyLens = robotHuskLens.initHuskyLens(robot.getHuskyLens(), HuskyLens_Apollo.PropColor.RED);
        if (initHuskyLens == false)
        {
            telemetry.addLine("failed to init Husky lens");
        }
        robot.ServoInit();
        robot.SetAllDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetAllDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_USING_ENCODER);

        resetHeading();
        telemetry.addLine("robot finish init");
        telemetry.update();

        waitForStart();

        TimeOut.reset();
        /*
        while ((detectedPropPos == null ) && (opModeIsActive() == true) && (TimeOut.seconds() < propDetectionTimeOut))
        {
            detectedPropPos = robotHuskLens.detectPropPos();
        }
        if (TimeOut.seconds() >= propDetectionTimeOut)
        {
            detectedPropPos = HuskyLens_Apollo.PropPos.RIGHT;
            //Log
            telemetry.addLine("failed ta detect Prop");
            sleep(1000);
        }
        telemetry.addData("Prop pos is " , detectedPropPos.toString());
        telemetry.update();
        while (opModeIsActive() == true)
        {
            //driveToProb(detectedPropPos);
        }
         */
        //driveStraight(DRIVE_SPEED, 23 * 5, 0 );


        driveStraight(DRIVE_SPEED,30,0);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.DUMP_SERVO, RobotHardware_apollo.SERVO_POS.DUMP_SERVO_OPEN.Pos);
        /*
        robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION,0.5);
        robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.ZeroPowerBehavior.BRAKE);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
        driveStraight(DRIVE_SPEED,-7,0);
        turnToHeadingApollo(TURN_SPEED,-90);
        driveStraight(DRIVE_SPEED,-30,-90);
        turnToHeadingApollo(TURN_SPEED,0);
        driveStraight(DRIVE_SPEED,-15,0);
        turnToHeadingApollo(TURN_SPEED,90);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT,1500);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT,1);
        sleep(1000);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
        sleep(2000);
        driveStraight(DRIVE_SPEED,7,90);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
        sleep(2000);
        driveStraight(DRIVE_SPEED,-7,90);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.PLANE_SERVO_CLOSE.Pos);
        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
        sleep(1000);
        robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT,0);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT,1);
        sleep(1000);
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


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            Log.d(TAG_DRIVE, "drive Straight; maxDriveSpeed: " + maxDriveSpeed + " distance: "+ distance + " heading: " + heading);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) + moveCounts;
            frontRightTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) + moveCounts;
            backLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) + moveCounts;
            backRightTarget = (int)robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE) + moveCounts;
            Log.d(TAG_DRIVE,"Current Position; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            Log.d(TAG_DRIVE,"Go To Position; front left: " + frontLeftTarget +
                    " front right "+ frontRightTarget +
                    " back left "+ backLeftTarget+
                    " back right "+ backRightTarget);
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightTarget);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) &&
                                robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) &&
                                    robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE))){

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            Log.d(TAG_DRIVE,"Stopped!!!!!!!!!!!!!!!!!!");
            Log.d(TAG_DRIVE,"Position at stop; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            //  sleep(3000);
            Log.d(TAG_DRIVE,"Position after stop;" +
                    " front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveRight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            Log.d(TAG_DRIVE, "drive right; maxDriveSpeed: " + maxDriveSpeed + "distance: " + heading);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (Math.abs(distance) * COUNTS_PER_INCH);
            frontLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) + moveCounts;
            frontRightTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) - moveCounts;
            backLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) - moveCounts;
            backRightTarget = (int)robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE) + moveCounts;
            Log.d(TAG_DRIVE,"Current Position; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                                  "front right " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                                  "back left " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) +
                                  "back right " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightTarget);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE))){

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveLeft(double maxDriveSpeed,
                           double distance,
                           double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            Log.d(TAG_DRIVE, "drive left; maxDriveSpeed: " + maxDriveSpeed + "distance:" + "distance: " + distance + "heading: "+ heading);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (Math.abs(distance) * COUNTS_PER_INCH);
            frontLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) - moveCounts;
            frontRightTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) + moveCounts;
            backLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) + moveCounts;
            backRightTarget = (int)robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE) - moveCounts;

            Log.d(TAG_DRIVE,"Current Position; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    "front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    "back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    "back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE)) ;
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightTarget);
             /*
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, 1000);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, -1000);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, -1000);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, 1000);
              */
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE))){

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeadingApollo(double maxTurnSpeed, double heading)
    {
        turnToHeading(maxTurnSpeed, heading);
        //double currentHeading = getRawHeading();
        getSteeringCorrection(heading,P_TURN_GAIN);
        if (Math.abs(headingError) >= HEADING_THRESHOLD)
        {
            Log.d(TAG_DRIVE,"i have missed the target heading and now i'm trying again");
            turnToHeading(TURN_SPEED_FIX,heading);
        }
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {

        Log.d(TAG_DRIVE,"turn To Heading; maxTurnSpeed: " + maxTurnSpeed + " heading: " + heading);
        Log.d(TAG_DRIVE,"real heading is " + getRawHeading());

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
        Log.d(TAG_DRIVE, "I got to the wanted heading " + heading);
        Log.d(TAG_DRIVE,"actual heading is "+ getRawHeading());
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double backLeftPower  = drive - turn;
        double backRightPower = drive + turn;
        double frontRightPower = drive + turn;
        double frontLeftPower = drive - turn;

        double maxFront = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        double maxBack = Math.max (Math.abs(backLeftPower), Math.abs(backRightPower));
        double max = Math.max(maxFront, maxBack);

        if (max > 1) {
            backLeftPower /= max;
            backRightPower /= max;
            frontLeftPower /= max;
            frontRightPower /= max;
        }
        Log.d(TAG_DRIVE, "Wheel turn is " + turn);
        Log.d(TAG_DRIVE,"Wheel Speeds is; " +
                " back Left Power is " + backLeftPower +
                " back Right Power is " + backRightPower +
                " front Right Power" + frontRightPower +
                " front Left Power" + frontLeftPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftPower);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
           Log.d(TAG_DRIVE,"Motion " + "Drive Straight " );
          //  telemetry.addData("Motion", "Drive Straight");
            //telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
           // telemetry.addData("Actual Pos L:R",  "%7d:%7d",      frontLeftDrive.getCurrentPosition(),
                    //frontRightDrive.getCurrentPosition());

        } else {
            Log.d(TAG_DRIVE, "Motion " +"Turning ");
           // telemetry.addData("Motion", "Turning");
        }
        Log.d(TAG_DRIVE, "Angle Targe;  Current " + targetHeading + " robot: " +  robotHeading);
      Log.d(TAG_DRIVE,"Error:Steer " + headingError + " robot: " + turnSpeed);
        //telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        //telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        //telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        //telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        double angles = robot.getImuRawHeading();
        Log.d(TAG_DRIVE,"robot angle. " + angles);
        return angles;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    public void driveToProb(HuskyLens_Apollo.PropPos probPos)
    {
        switch (probPos)
        {
            case UP:
                time.reset();
                driveStraight(DRIVE_SPEED, 23.5, 0.0);
                turnToHeading( TURN_SPEED, 180.0);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                //armServo.setPosition(0.25);
                //armGardServo.setPosition(0.3);
                sleep(600);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
                //armGardServo.setPosition(0.45);
                //armServo.setPosition(0.71);// Drive Forward 24"
                //sleep(3000);
                turnToHeading( TURN_SPEED, -90.0);
                driveStraight(DRIVE_SPEED, -34, -90.0);
                //sleep(3000);
                robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT, 300);
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_TO_POSITION);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, 1);
                //lift.setTargetPosition(300);
                //lift.setPower(1);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                //armServo.setPosition(0.25);
                //armGardServo.setPosition(0.15);
                Log.d(TAG_TIME, "the final time is " + time.milliseconds());
                break;
            case RIGHT:
                time.reset();
                driveStraight(DRIVE_SPEED, 23.5, 0.0);
                turnToHeading(TURN_SPEED, -90);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                //armServo.setPosition(0.25);
                //armGardServo.setPosition(0.3);
                sleep(600);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
                //armGardServo.setPosition(0.45);
                //armServo.setPosition(0.71);// Drive Forward 24"
                //sleep(3000);
                //turnToHeading( TURN_SPEED, -90.0);
                driveRight(DRIVE_SPEED, -20, -90);
                driveStraight(DRIVE_SPEED, -34, -90.0);
                //sleep(3000);
                robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT, 300);
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_TO_POSITION);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, 1);
                //lift.setTargetPosition(300);
                //lift.setPower(1);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                //armServo.setPosition(0.25);
                //armGardServo.setPosition(0.15);
                Log.d(TAG_TIME, "the final time is " + time.milliseconds());
                break;
            case LEFT:
                time.reset();
                driveStraight(DRIVE_SPEED, 23.5, 0.0);
                turnToHeading(TURN_SPEED, 90);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                //armServo.setPosition(0.25);
                //armGardServo.setPosition(0.3);
                sleep(600);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
                //armGardServo.setPosition(0.45);
                //armServo.setPosition(0.71);// Drive Forward 24"
                turnToHeading(TURN_SPEED, -90);
                //sleep(3000);
                //turnToHeading( TURN_SPEED, -90.0);
                driveStraight(DRIVE_SPEED, -34, -90.0);
                driveLeft(DRIVE_SPEED, -20, -90);
                //sleep(3000);
                robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT, 1000);
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_TO_POSITION);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, 1);
                //lift.setTargetPosition(300);
                //lift.setPower(1);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                //armServo.setPosition(0.25);
                //armGardServo.setPosition(0.15);
                Log.d(TAG_TIME, "the final time is " + time.milliseconds());

        }
    }
    //private void
}
