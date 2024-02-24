/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="TeleOp apollo", group="TeleOp")
//@Disabled
public class BasicOpMode_apollo_better extends OpMode {

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime TimeOut = new ElapsedTime();
    private double TimeOutSec = 3;
    //double Test = 0;
    public static double collectionSpeed = 0.8;
    boolean press = false;
    boolean pressCollection = false;
    boolean pressCollectionConf = false;
    double old_ARM_SERVO_DUMP_POS = 0;
    double armServoPos = 0;
    double armGardServoPos = 0;
    boolean pressCollectionServo = false;
    boolean pressCollectionLift = false;
    boolean pressLift = false;
    boolean pressDrive = false;
    boolean LIFT_IsBusy;
    final int FIRST_LIFT = 1000;
    double servoCurrentPosition;
    final int SECOND_LIFT = 1710;
    final int THIRD_LIFT = 2565;
    final int FOURTH_LIFT = 2141;
    int liftMaxHight = 3420;
    final double POWER_LIFT = 1;
    double liftPower = 0;
    boolean inPosition = false;
    final String TAG_LIFT = "Lift";
    final String TAG_DRIVE = "Drive";
    final String TAG_COLLECTION = "Collection";
    final String TAG_COLLECTION_THREAD = "CollectionThread";
    final String TAG_LIFT_THREAD = "LiftThread";
    int currentPosition;
    final double LIFT_TIMEOUT_SEC = 5;
    final double LIFT_TIMEOUT_STAY_SEC = 2;
    boolean controlMod = false;
    boolean upSideDownMod = false;
    boolean fieldCentricDrive = false;
    boolean doNotGoDownMod = false;
    collectThread collectThread = new collectThread();
    LiftThread liftTread = new LiftThread();

    enum LiftState {STOP,
        MANUAL_CONTROL,
        AUTO_CONTROL_WAIT_FOR_BUSY,
        AUTO_CONTROL_IN_POSITION,
        AUTO_CONTROL_ERROR,
        RESETING_INCODER};
    int pos;
    enum LiftStopStat
    {
        OPEN,
        CLOSE
    }
    enum LiftXStat
    {
        FIRST,
        SECOND
    }
    LiftXStat liftXStat;
    LiftStopStat liftStopStat;
    boolean stayInPosIsActive = false;
    //ConceptTensorFlowObjectDetection_Apollo detection;
    RobotHardware_apollo robot = new RobotHardware_apollo();
    RobotHardware_apollo_FtcLib robot_Ftclib = new RobotHardware_apollo_FtcLib();

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        //old_ARM_SERVO_DUMP_POS = robot.ARM_SERVO_DUMP_POS;
        old_ARM_SERVO_DUMP_POS = RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos;
        robot.init(hardwareMap,false,false);
        boolean initIMU =  robot_Ftclib.init(hardwareMap, true);
        if (initIMU == false)
        {
            telemetry.addLine("failed to init Imu (stop!!!!!!!!!!!)");
        }
        else
        {
            telemetry.addLine("int Imu succeeded");
        }
        robot.ServoInit();
        liftStopStat = LiftStopStat.OPEN;
        liftXStat = LiftXStat.FIRST;
        robot.plane_state = RobotHardware_apollo.PLANE_STATE.CLOSE;
        //robot_Ftclib.SetAllMotorsZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_USING_ENCODER);

        //detection = new ConceptTensorFlowObjectDetection_Apollo(this);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive"); //0
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive"); //1
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive"); //2
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");//3
        rightCollection = hardwareMap.get(DcMotor.class, "right_collection");//0
        leftCollection = hardwareMap.get(DcMotor.class, "left_collection");//1
        lift = hardwareMap.get(DcMotor.class, "lift");//2
        touchSensor1 = hardwareMap.get(TouchSensor.class, "sensor_touch1");
        touchSensor2 = hardwareMap.get(TouchSensor.class, "sensor_touch2");
        collectionServo = hardwareMap.get(Servo.class, "collection_servo");
        collectionGardServo = hardwareMap.get(Servo.class, "collection_gard_servo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightCollection.setDirection(DcMotorSimple.Direction.FORWARD);
        leftCollection.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionServo.setDirection(Servo.Direction.FORWARD);
        collectionGardServo.setDirection(Servo.Direction.FORWARD);


         */


        telemetry.addData("Status", "Initialized");
        telemetry.update();


    }

    public void start()
    {
        runtime.reset();
        collectThread.start();
        liftTread.start();
    }
    @Override
    public void loop() {

        drive();
            /*
            if(robot.IsPressed(RobotHardware_apollo.DriveMotors.TOUCH_SENSOR1) == true)
            {
                telemetry.addData("Touch Sensor1", "Is Pressed");
            }
            else
            {
                telemetry.addData("Touch Sensor1", "Is Not Pressed");
            }
            if(robot.IsPressed(RobotHardware_apollo.DriveMotors.TOUCH_SENSOR2) == true)
            {                telemetry.addData("Touch Sensor2", "Is Pressed");
            }
            else
            {
                telemetry.addData
                ("Touch Sensor2", "Is Not Pressed");
            }

             */
        //boolean collectionSpeedMore = gamepad2.right_bumper;
        //boolean collectionSpeedLess = gamepad2.left_bumper;

        //  if (collectionSpeedMore == true)
        {
            // if (press == false)
            {
                //press = true;
                // if (collectionSpeed < 1)
                {
                    // collectionSpeed += 0.1;
                }
            }
        }

        // if (collectionSpeedLess== true)
        {
            // if (press == false)
            {
                //press = true;
                // if (collectionSpeed > 0.1)
                {
                    // collectionSpeed -= 0.1;
                }
            }
        }

        //if ((collectionSpeedLess == false) && (collectionSpeedMore == false))
        {
            //press = false;
        }






        // Show the elapsed game time and wheel power.

        /*
        if (true == inPosition )
        {
            telemetry.addLine("lift in position!!!");
        }
        //detection.detect();
        telemetry.addData("ARM_SERVO_DUMP_POS is " , robot.ARM_SERVO_DUMP_POS);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("MAXH is","(%.2f)" + liftMaxHight);
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        //telemetry.addData("side power", "(%.2f)",sidePower);
        telemetry.addData("lift Pos is ", "(%.2f)", robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
        telemetry.addData("collectionSpeed", "(%.2f)", collectionSpeed);
        telemetry.addData("servo pos is", "(%.2f)", robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO));
         */
        //telemetry.addLine("heading is " + robot_Ftclib.getRobotYawPitchRollAngles());
        telemetry.addData("gard stat is  " + robot.armServoGardState +  " gard Pos is " , "(%.4f)" , armGardServoPos);
        telemetry.addData("arm stat is  " + robot.armServoState + " arm Pos is " , "(%.4f)" , armServoPos);
        telemetry.addData("gard current pos is " , "(%.4f)" , robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO));
        telemetry.addData("arm current Pos is " , "(%.4f)" , robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO));
        telemetry.addLine("field Centric Drive stats is " + fieldCentricDrive);
        telemetry.addLine("control Mod stats is " + controlMod);
        telemetry.addLine("up Side Down Mod stats is " + upSideDownMod);
        telemetry.addLine("lift stop servo stat is " + liftStopStat);
        telemetry.addLine("lift Pos is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
        telemetry.update();

    }
    public void stop()
    {
        while ((liftTread.isAlive()) || (collectThread.isAlive()))
        {
            liftTread.interrupt();
            collectThread.interrupt();
        }

    }
    private void drive()
    {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        double forwardSpeed;
        double turnSpeed;
        double strafeSpeed;
        if (!upSideDownMod)
        {
            forwardSpeed   = gamepadEx1.getLeftY();  // Note: pushing stick forward gives negative value
            strafeSpeed =  gamepadEx1.getLeftX();
            turnSpeed     =  -gamepadEx1.getRightX();
        }
        else
        {
            forwardSpeed   = -gamepadEx1.getLeftY();  // Note: pushing stick forward gives negative value
            strafeSpeed =  gamepadEx1.getLeftX();
            turnSpeed     =  -gamepadEx1.getRightX();
        }
        if ((gamepadEx1.wasJustPressed(GamepadKeys.Button.BACK)) && (!gamepadEx1.isDown(GamepadKeys.Button.B)) && (!gamepadEx1.isDown(GamepadKeys.Button.A)))
        {
            robot_Ftclib.ResetYaw();
            fieldCentricDrive = !fieldCentricDrive;
        }
        double heading = robot_Ftclib.getYaw();
        if (gamepad1.y)
        {
            if(!pressDrive)
            {
                pressDrive = true;
                controlMod = !controlMod;
            }
            //upSideDownMod = !upSideDownMod;

        }
        else
        {
            pressDrive = false;
        }
        if (!fieldCentricDrive)
        {
            if (controlMod == true)
            {
                robot_Ftclib.driveRobotCentric(strafeSpeed/2, forwardSpeed/2, turnSpeed/2);
            }
            else
            {
                robot_Ftclib.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
            }
        }
        else
        {
            if (controlMod == true)
            {
                robot_Ftclib.driveFieldCentric(strafeSpeed/2, forwardSpeed/2, turnSpeed/2,heading);
            }
            else
            {
                robot_Ftclib.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed,heading);
            }
        }
    }

    public class collectThread extends Thread
    {
        public collectThread()
        {
            this.setName("CollectThread");
            Log.d(TAG_COLLECTION_THREAD, "start collectThread");
        }


        @Override

        public void run()
        {
            try
            {

                while (!isInterrupted())
                {
                    /*
                    double collectionR = gamepad1.left_trigger; //R-reverse: pixel emission
                    double collectionF = gamepad1.right_trigger; // F-forward: pixel collection
                    boolean collectPixel = gamepad2.right_bumper;
                    boolean dumpPixel =  gamepad2.left_bumper;
                     */
                    boolean collectionR = gamepad2.left_bumper; //R-reverse: pixel emission left_bumper
                    double collectionF = gamepad2.left_trigger; // F-forward: pixel collection
                    boolean collectPixel = gamepad2.right_bumper;
                    float dumpPixel =  gamepad2.right_trigger;
                    //float gatePositionOpenClose = gamepad2.left_trigger;
                    //boolean gatePositionOpen = gamepad2.left_bumper;

                    //boolean doNotGoDownSwitch = gamepad2.right_bumper;


                    /*
                    if (gatePositionOpenClose > 0.3)
                    {
                        robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN_CLOSE;
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, robot.ARM_SERVO_GARD_OPEN_CLOSE_POS);
                    }
                    else if (gatePositionOpen == true)
                    {
                        robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN;
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, robot.ARM_SERVO_GARD_OPEN_POS);
                    }


                     */
                    /*
                    if (gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3)
                    {
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_OPEN.Pos);
                        liftStopStat = LiftStopStat.OPEN;
                    }
                    else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.3)
                    {
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_CLOSE.Pos);
                        liftStopStat = LiftStopStat.CLOSE;
                    }
                    */
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))
                        {
                        if (robot.plane_state == RobotHardware_apollo.PLANE_STATE.OPEN)
                        {
                            robot.plane_state = RobotHardware_apollo.PLANE_STATE.CLOSE;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO, RobotHardware_apollo.SERVO_POS.PLANE_SERVO_CLOSE.Pos);
                        }
                        else if(robot.plane_state == RobotHardware_apollo.PLANE_STATE.CLOSE)
                        {
                            robot.plane_state = RobotHardware_apollo.PLANE_STATE.OPEN;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO, RobotHardware_apollo.SERVO_POS.PLANE_SERVO_OPEN.Pos);
                        }
                    }
                    if (gamepad1.dpad_down && gamepad1.back)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos -= 0.01;
                            RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos -= 0.01;
                            Log.d(TAG_COLLECTION, "ARM_SERVO_DUMP_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            armServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos;
                        }
                        //RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos -= 0.025;
                        //robot.ARM_SERVO_DUMP_POS -= 0.025;

                    }
                    else if (gamepad1.dpad_up && gamepad1.back)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos += 0.01;
                            RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos += 0.01;
                            //robot.ARM_SERVO_DUMP_POS += 0.025;
                            Log.d(TAG_COLLECTION, "ARM_SERVO_DUMP_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            armServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos;

                        }
                    }
                    else if (gamepad1.dpad_left && gamepad1.back)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            switch (robot.armServoGardState)
                            {
                                case OPEN:
                                {
                                    RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos -= 0.001;
                                    Log.d(TAG_COLLECTION, "ARM_SERVO_GARD_OPEN_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                                    armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos;
                                }
                                break;
                                case CLOSE:
                                {
                                    RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos -= 0.001;
                                    Log.d(TAG_COLLECTION, "ARM_SERVO_GARD_CLOSE_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                                    armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos;
                                }
                                break;
                                case OPEN_CLOSE:
                                {
                                    RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos -= 0.001;
                                    Log.d(TAG_COLLECTION, "ARM_SERVO_GARD_OPEN_CLOSE_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                                    armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos;
                                }
                                break;
                            }
                        }
                    }
                    else if (gamepad1.dpad_right && gamepad1.back)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            switch (robot.armServoGardState)
                            {
                                case OPEN: {
                                    RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos += 0.001;
                                    Log.d(TAG_COLLECTION, "ARM_SERVO_GARD_OPEN_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                                    armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos;
                                }
                                break;
                                case CLOSE: {
                                    RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos += 0.001;
                                    Log.d(TAG_COLLECTION, "ARM_SERVO_GARD_CLOSE_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                                    armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos;
                                }
                                break;
                                case OPEN_CLOSE: {
                                    RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos += 0.001;
                                    Log.d(TAG_COLLECTION, "ARM_SERVO_GARD_OPEN_CLOSE_POS is " + RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                                    armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos;
                                }
                                break;
                            }
                        }
                    }
                    else if (gamepad1.dpad_up)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            double currentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO);
                            currentPosition += 0.01;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, currentPosition);
                        }
                        //RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos -= 0.025;
                        //robot.ARM_SERVO_DUMP_POS -= 0.025;

                    }
                    else if (gamepad1.dpad_down)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            double currentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO);
                            currentPosition -= 0.01;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, currentPosition);
                        }
                        //RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos -= 0.025;
                        //robot.ARM_SERVO_DUMP_POS -= 0.025;

                    }
                    else if (gamepad1.dpad_right)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            servoCurrentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO);
                            servoCurrentPosition += 0.01;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, servoCurrentPosition);
                        }
                    }
                    else if (gamepad1.dpad_left)
                    {
                        if (pressCollectionConf)
                        {
                            pressCollectionConf = false;
                            servoCurrentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO);
                            servoCurrentPosition -= 0.01;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, servoCurrentPosition);
                        }
                    }
                    else
                    {
                        pressCollectionConf = true;
                    }
                    if (collectPixel == true)
                    {
                        if (pressCollectionServo == false)
                        {
                            pressCollectionServo = true;
                            CollectPixel();

                            liftTread.goTo(0);
                        }
                    }

                    else if (dumpPixel > 0.3)
                    {
                        if (pressCollectionServo == false)
                        {
                            pressCollectionServo = true;
                            DumpPixel();
                        /*
                        else if (armServoState == ArmServoState.OPEN)
                        {
                            armServoState = ArmServoState.CLOSE;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.COLLECTION_GARD_SERVO, 0.45);
                        }
                        */
                        }
                    }
                    else if(gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                    {
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                    }
                    else
                    {
                        servoStayInPos();
                        pressCollectionServo = false;
                    }


                    if (collectionF != 0)
                    {
                        collectionMotor(collectionSpeed);
                    }
                    else if (collectionR == true)
                    {
                        if (pressCollection == false)
                        {
                            pressCollection = true;
                            CollectPixel();
                            //liftTread.goTo(0);

                        }
                        collectionMotor(-collectionSpeed);
                    }
                    //else if ((detection.tfod.getRecognitions().size() < 0))
                    else
                    {
                        collectionMotor(0);
                    }
                }
            }
            catch (Exception e)
            {
                Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
            }

        }


        public void servoStayInPos()
        {
            //if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) >= 200)
            //{
                servoCurrentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO);
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO,servoCurrentPosition);
                /*
                if(robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.OPEN)
                {
                    //robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN;
                    //armServoGardState = ArmServoGardState.OPEN;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                }
                else if (robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.CLOSE)
                {
                    //robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.CLOSE;
                    //armServoGardState = ArmServoGardState.CLOSE;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                }

                 */
            //}
        }
        public void collectionMotor (double speed)
        {
            robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, speed);
        }
        public void CollectPixel()
        {
            robot.armServoState = RobotHardware_apollo.ArmServoState.COLLECT;
            //74

            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO) != RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos)
            {
                robot.armServoState = RobotHardware_apollo.ArmServoState.COLLECT;
                // log
                //armServoState = ArmServoState.COLLECT;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
            }
            //if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO) != RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos)
            //{
                //log
                robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN;
                //armServoGardState = ArmServoGardState.OPEN;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
            //}
        }
        public void DumpPixel() throws InterruptedException {
            //if(robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) >= FIRST_LIFT)
            //{

                Log.d(TAG_COLLECTION,"armServoState is before " + robot.armServoState);
                Log.d(TAG_COLLECTION,"armServoGardState is before " + robot.armServoGardState);
                if(robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.CLOSE)
                {
                    if (robot.armServoState != RobotHardware_apollo.ArmServoState.COLLECT) {
                        robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN_CLOSE;
                        //armServoGardState = ArmServoGardState.OPEN_CLOSE;
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);
                        try
                        {
                            Thread.sleep(750);
                        }  catch (Exception e)
                        {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                        robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                    }
                }
                else if(robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.OPEN_CLOSE)
                {
                    robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN;
                    //armServoGardState = ArmServoGardState.OPEN;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
                }
                else if (robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.OPEN)
                {
                    robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.CLOSE;
                    //armServoGardState = ArmServoGardState.CLOSE;
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
                }
                robot.armServoState = RobotHardware_apollo.ArmServoState.DUMP;
                if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO) != RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos)
                {
                    robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                }

                Log.d(TAG_COLLECTION,"armServoState is next " + robot.armServoState);
                Log.d(TAG_COLLECTION,"armServoGardState is next " + robot.armServoGardState);
            //}
        }
    }
    // ezra test check github
    public class LiftThread extends Thread
    {
        private LiftState liftState;
        private ElapsedTime liftTime = new ElapsedTime();
        private double timerTimeoutInSeconds;

        public LiftThread()
        {
            this.setName("LiftThread");
            Log.d(TAG_COLLECTION_THREAD, "start Lift thread");
        }
        public void run()
        {
            liftState = LiftState.STOP;
            liftTime.reset();
            Log.d(TAG_LIFT,"start");
            while (!isInterrupted())
            {
                boolean liftUp = gamepad2.dpad_up;
                boolean liftDown = gamepad2.dpad_down;
                boolean liftPositionY = gamepad2.y;
                boolean liftPositionA = gamepad2.a;
                boolean liftPositionB = gamepad2.b;
                boolean liftPositionX = gamepad2.x;
                boolean liftStop = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
                boolean liftReset = gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
                Log.d(TAG_LIFT,"liftPositionY " + liftPositionY);
                Log.d(TAG_LIFT,"liftPositionA " + liftPositionA);
                Log.d(TAG_LIFT,"liftPositionB " + liftPositionB);
                Log.d(TAG_LIFT,"liftPositionX " + liftPositionX);
                Log.d(TAG_LIFT,"pos is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                if (liftStop)
                {
                    switch (liftStopStat)
                    {
                        case OPEN:
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_CLOSE.Pos);
                            liftStopStat = LiftStopStat.CLOSE;
                            Log.d(TAG_LIFT,"lift Pos at lock is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                        break;
                        case CLOSE:
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_OPEN.Pos);
                            liftStopStat = LiftStopStat.OPEN;
                        break;
                        default:

                        break;

                    }
                }
                if (liftReset)
                {
                    liftState = LiftState.RESETING_INCODER;
                    liftPower = 0;
                    robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, liftPower);
                    robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND, liftPower);
                    robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addLine("lift is at pos 0");
                }
                if (liftUp == false)
                {
                    resetIncoder();
                }
                if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY)
                {
                    if (isLiftBusy() == true)
                    {
                        /*
                        if (stayInPosIsActive == true)
                        {
                            Log.d(TAG_LIFT, "stay in position " + pos + " ; current position is " + lift.getCurrentPosition());
                            stayInPosIsActive = false;
                        }
                        else
                        {*/

                        Log.d(TAG_LIFT, "run to position " + pos + " ; current position is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                        //}

                    }
                    if (isLiftBusy() == false)
                    {
                        EndOfGetToPos();
                    }
                }
                if (liftUp == true)
                {
                    if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) < liftMaxHight)
                    {
                        setGateServoToClose();
                        liftState = LiftState.MANUAL_CONTROL;
                        Log.d(TAG_LIFT, "lift up pos " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftPower = POWER_LIFT;
                        inPosition = false;
                    }
                    else
                    {
                        if (inPosition == false)
                        {
                            liftPower = 0;
                            //stayInPos();
                        }
                    }

                }
                else if (liftDown == true)
                {
                    if (liftState != LiftState.RESETING_INCODER)
                    {
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, ARM_SERVO_COLLECT_POS);
                        //collectThread.CollectPixel();
                        liftState = LiftState.MANUAL_CONTROL;
                        Log.d(TAG_LIFT, "lift down pos " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftPower = -POWER_LIFT;
                        inPosition = false;
                        /*
                        if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) >= 50)
                        {
                        }
                        else
                        {
                            double distanceToZero = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);
                            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT , (int) (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) - distanceToZero));
                            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_TO_POSITION);
                            liftPower = -POWER_LIFT;
                            robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT , liftPower);
                            robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND , liftPower);
                            //liftPower = 0;
                            if (inPosition == false)
                            {
                                //stayInPos();
                            }
                        }

                         */
                    }
                }
                else if (liftPositionY == true)
                {
                    if (pressLift == false)
                    {
                        //collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        try {
                            goTo(FIRST_LIFT);
                            robot.armServoState = RobotHardware_apollo.ArmServoState.DUMP;
                            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO) != RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos)
                            {
                                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            }
                        }  catch (Exception e)
                        {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }

                    }

                }
                else if ((liftPositionB == true) && (!gamepadEx2.isDown(GamepadKeys.Button.START)))
                {
                    if (pressLift == false)
                    {
                        //collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        try {
                            goTo(SECOND_LIFT);
                            robot.armServoState = RobotHardware_apollo.ArmServoState.DUMP;
                            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO) != RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos)
                            {
                                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            }
                        }  catch (Exception e)
                        {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                    }

                }
                else if ((liftPositionA == true) && (!gamepadEx2.isDown(GamepadKeys.Button.START)))
                {
                    if (pressLift == false)
                    {
                        //collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        try {
                            goTo(THIRD_LIFT);
                            robot.armServoState = RobotHardware_apollo.ArmServoState.DUMP;
                            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO) != RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos)
                            {
                                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);
                            }
                        }  catch (Exception e)
                        {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                    }

                }
                else if (liftPositionX == true)
                {
                    if (pressLift == false)
                    {
                        //collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        try {
                            switch (liftXStat)
                            {
                                case FIRST:
                                    goTo(FOURTH_LIFT);
                                    setGateServoToClose();
                                    collectThread.DumpPixel();
                                    liftXStat = LiftXStat.SECOND;
                                break;
                                case SECOND:
                                    goTo(250);//0/250
                                    //setGateServoToClose();
                                    LIFT_IsBusy = robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
                                    while ((LIFT_IsBusy) && (TimeOut.seconds() < TimeOutSec))
                                    {
                                        LIFT_IsBusy = robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
                                    }
                                    robot.SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_CLOSE.Pos);
                                    liftStopStat = LiftStopStat.CLOSE;
                                    Log.d(TAG_LIFT,"lift Pos at lock is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                                    //collectThread.DumpPixel();
                                break;
                            }
                        }  catch (Exception e)
                        {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                    }

                }
                else
                {
                    if ((inPosition == false) && (isLiftBusy() == false))
                    {
                        if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) >= 0)
                        {
                            stayInPos();
                        }

                    }
                    else if (isLiftBusy() == false)
                    {
                        liftState = LiftState.STOP;
                    }

                }
                if ((liftPositionA == false) && (liftPositionB == false) && (liftPositionX == false) && (liftPositionY == false) &&(gamepad2.right_bumper==false) && (gamepad2.left_bumper==false))
                {
                    pressLift = false;
                }

                if(liftPower != 0)
                {
                    Log.d(TAG_LIFT, "lift Power is " + liftPower);
                    Log.d(TAG_LIFT, "lift mode is  " + robot.lift.getMode());
                    Log.d(TAG_LIFT, "current position " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));

                }
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT , liftPower);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND , liftPower);


            }
            Log.d(TAG_LIFT,"finish");

        }
        public void resetIncoder()
        {

            /*
            if ((robot.IsPressed(RobotHardware_apollo.DriveMotors.TOUCH_SENSOR1) == true) && (robot.IsPressed(RobotHardware_apollo.DriveMotors.TOUCH_SENSOR2)))
            {
                liftState = LiftState.RESETING_INCODER;
                liftPower = 0;
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, liftPower);
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("lift is at pos 0");
            }

             */
        }
        public void goTo(int Pos) throws InterruptedException {
            if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY)
            {
                if (isLiftBusy() == true)
                {
                    liftrest();
                }
            }
            if (Pos != 0){
                setGateServoToClose();
            }
            if (Pos == 0)
            {
                try {
                    double currentPosition = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);
                    if (currentPosition < SECOND_LIFT)
                    {
                        Thread.sleep(1000);
                    }
                }  catch (Exception e)
                {
                    Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                }
            }
            telemetry.addData("run to position ", "%d", Pos);
            Log.d(TAG_LIFT, "run to position " + Pos);
            pos = Pos;
            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_USING_ENCODER);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT , Pos);
            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_TO_POSITION);
            liftPower = POWER_LIFT;
            if (currentPosition > pos)
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT ,liftPower);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND ,-liftPower);
            }
            else
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT ,liftPower);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND ,liftPower);
            }
            inPosition = false;
            liftTime.reset();
            liftState = LiftState.AUTO_CONTROL_WAIT_FOR_BUSY;
            timerTimeoutInSeconds = LIFT_TIMEOUT_SEC;
        }

        private void stayInPos()
        {
            Log.d(TAG_LIFT,"stayInPos start");
            //liftState != LiftState.RESETING_INCODER ||
            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) > 20)
            {
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_USING_ENCODER);
                currentPosition = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);;
                currentPosition += 10;
                Log.d(TAG_LIFT, "stay in pos start, current Position is " + currentPosition);
                pos = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);;
                //stayInPosIsActive = true;
                robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT , currentPosition);
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_TO_POSITION);
                liftPower = POWER_LIFT;
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, liftPower);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND, liftPower);
                liftTime.reset();
                liftState = LiftState.AUTO_CONTROL_WAIT_FOR_BUSY;
                timerTimeoutInSeconds = LIFT_TIMEOUT_STAY_SEC;
            }
            else
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, 0);
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT_SECOND, 0);
            }



        }
        private boolean isLiftBusy()
        {
            boolean isLiftBusy = false;
            robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
            if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY)
            {
                if ((robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT) == true) && (liftTime.seconds() < timerTimeoutInSeconds))
                {
                    isLiftBusy = true;
                }

            }
            return (isLiftBusy);
        }
        private void EndOfGetToPos()
        {
            if (liftTime.seconds() > timerTimeoutInSeconds)
            {

                Log.d(TAG_LIFT, "stopped due to time out. stopped at " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                liftState = LiftState.AUTO_CONTROL_ERROR;
                inPosition = true;

            }
            else
            {
                Log.d(TAG_LIFT, "Final Position is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                Log.d(TAG_LIFT, "timer in milliseconds is " + liftTime.milliseconds() + " timer in seconds is " + liftTime.seconds());
                inPosition = true;
                liftState = LiftState.AUTO_CONTROL_IN_POSITION;
            }
            robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.ZeroPowerBehavior.BRAKE);
        }
        private void liftrest()
        {
            stayInPos();
            Log.d(TAG_LIFT, "lift rest");
        }

        private void setGateServoToClose()
        {
            robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.CLOSE;
            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);
        }
    }



}
