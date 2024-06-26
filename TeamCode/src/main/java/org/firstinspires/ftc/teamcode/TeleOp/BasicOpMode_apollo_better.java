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

package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoDrive.AutoDriveApollo;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotHardware_apollo;
import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotHardware_apollo_FtcLib;
import org.firstinspires.ftc.teamcode.RobotHardware_apollo.RobotMove_apollo;


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
    Gamepad.RumbleEffect liftLockRumble;


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime TimeOut = new ElapsedTime();
    private double TimeOutSec = 3;
    //double Test = 0;
    public static double collectionSpeed = 2600;
    public static double collectionBackSpeed = 1950;
    boolean press = false;
    boolean pressCollection = false;
    boolean pressCollectionConf = false;
    boolean pressDrone = false;
    double armServoPos = 0;
    double armGardServoPos = 0;
    boolean pressCollectionServo = false;
    boolean pressCollectionLift = false;
    boolean pressLift = false;
    boolean pressDrive = false;
    boolean LIFT_IsBusy;
    final int FIRST_LIFT = 662;
    double servoCurrentPosition;
    final int SECOND_LIFT = 860;
    final int THIRD_LIFT = 1630;
    final int FOURTH_LIFT = 2200;
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
    public static boolean upSideDownMod = false;
    boolean fieldCentricDrive = false;
    double heading;
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
    enum LiftXStat
    {
        FIRST,
        SECOND
    }
    LiftXStat liftXStat;
    boolean stayInPosIsActive = false;
    //ConceptTensorFlowObjectDetection_Apollo detection;
    //RobotHardware_apollo robot = new RobotHardware_apollo();
    RobotMove_apollo robot = new RobotMove_apollo();
    RobotHardware_apollo_FtcLib robot_Ftclib = new RobotHardware_apollo_FtcLib();

    @Override
    public void init() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean initIMU = robot.Robot.init(hardwareMap,false,true);
        robot_Ftclib.init(hardwareMap, false);
        if (initIMU == false)
        {
            telemetry.addLine("failed to init Imu (stop!!!!!!!!!!!)");
        }
        else
        {
            telemetry.addLine("int Imu succeeded");
        }
        robot.Robot.ServoInit();
        liftXStat = LiftXStat.FIRST;
        //robot.Robot.plane_state = RobotHardware_apollo.PLANE_STATE.CLOSE;
        robot_Ftclib.SetAllMotorsZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.Robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_USING_ENCODER);



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
        telemetry.addData("gard stat is  " + robot.Robot.armGardState +  " gard Pos is " , "(%.4f)" , armGardServoPos);
        telemetry.addData("arm stat is  " + robot.Robot.armState + " arm Pos is " , "(%.4f)" , armServoPos);
        telemetry.addData("gard current pos is " , "(%.4f)" , robot.Robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO));
        telemetry.addData("arm current Pos is " , "(%.4f)" , robot.Robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO));
        */
        if (fieldCentricDrive)
        {
            telemetry.addLine("<h1> field Centric Drive stats is " + fieldCentricDrive + "</h1>");
            telemetry.addLine("<h1> heading is " + heading + "</h1>");
        }
        if (controlMod)
        {
            telemetry.addLine("<h1>control Mod stats is SLOW</h1>");
        }
        else
        {
            telemetry.addLine("<h1>control Mod stats is FAST</h1>");
        }
        //telemetry.addLine("up Side Down Mod stats is " + upSideDownMod);
        if (robot.Robot.liftLockStat == RobotHardware_apollo.LiftLockStat.LOCKED)
        {
            telemetry.addLine("<h1>lift stop servo stat is CLOSED</h1>");
        }
        else
        {
            telemetry.addLine("<h1>lift stop servo stat is OPEN</h1>");
        }
        //double AMP = robot.Robot.GetCurrent(RobotHardware_apollo.DriveMotors.COLLECTION);
        //telemetry.addData("AMP" , AMP);
        //telemetry.addLine("lift stop servo stat is " + robot.Robot.liftLockStat);
        //telemetry.addLine("lift Pos is " + robot.GetPosMotor.lift());
        //+
        // telemetry.addLine("lift zero power behavior is " + robot.Robot.GetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT));
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
            //robot.Robot.ResetYaw();
            robot.Robot.ResetYaw();
            fieldCentricDrive = !fieldCentricDrive;
        }
        heading = robot.Robot.getImuRawHeading();
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
        if(gamepad1.a)
        {
            robot_Ftclib .driveRobotCentric(0,
                    -0.25,
                    0);
        }
        else if (!fieldCentricDrive)
        {
            if (controlMod == true)
            {
                robot_Ftclib.driveRobotCentric(
                        strafeSpeed/2,
                        forwardSpeed/3,
                        turnSpeed/3);
            }
            else
            {
                robot_Ftclib.driveRobotCentric(
                        strafeSpeed,
                        forwardSpeed,
                        turnSpeed);
            }
        }
        else
        {
            if (controlMod)
            {
                robot_Ftclib.driveFieldCentric(
                        strafeSpeed/2,
                        forwardSpeed/3,
                        turnSpeed/3,
                        heading

                );
            }
            else
            {
                robot_Ftclib.driveFieldCentric(
                        strafeSpeed,
                        forwardSpeed,
                        turnSpeed,
                        heading
                );
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
                    boolean pixelCollection = gamepad2.left_bumper;
                    double pixelEmission = gamepad2.left_trigger;
                    boolean collectPixel = gamepad2.right_bumper;
                    float dumpPixel =  gamepad2.right_trigger;
                    if (gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))
                    {
                       lunchDrone();
                        gamepad2.rumble(1, 1, 500);
                    }
                    if (gamepad2.share)
                    {
                        if (!pressDrone)
                        {
                            pressDrone = true;
                            robot.MoveServo.closeGard();
                        }

                    }
                    else
                    {
                        pressDrone = false;
                    }
                    if (gamepad1.dpad_down)
                    {
                        if (!pressCollectionConf)
                        {
                            if (gamepad1.back)
                            {
                                RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos -= 0.01;
                                RobotHardware_apollo.SERVO_POS.ARM_COLLECT.Pos -= 0.01;
                                Log.d(TAG_COLLECTION, "ARM_DUMP is " + RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos);
                                robot.MoveServo.dumpPixel();
                                armServoPos = RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos;
                            }
                            else
                            {
                                double currentPosition = robot.GetPosServo.arm();
                                currentPosition -= 0.01;
                                robot.SetPosServo.arm(currentPosition);
                            }
                            pressCollectionConf = true;
                        }

                    }
                    else if (gamepad1.dpad_up)
                    {
                        if (!pressCollectionConf)
                        {
                            if (gamepad1.back)
                            {
                                RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos += 0.01;
                                RobotHardware_apollo.SERVO_POS.ARM_COLLECT.Pos += 0.01;
                                //robot.ARM_SERVO_DUMP_POS += 0.025;
                                Log.d(TAG_COLLECTION, "ARM_DUMP is " + RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos);
                                robot.MoveServo.dumpPixel();
                               armServoPos = RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos;
                            }
                            else
                            {
                                double currentPosition = robot.GetPosServo.arm();
                                currentPosition += 0.01;
                                robot.SetPosServo.arm(currentPosition);
                            }
                            pressCollectionConf = true;

                        }
                    }
                    else if (gamepad1.dpad_left)
                    {
                        if (!pressCollectionConf)
                        {
                            if (gamepad1.back)
                            {
                                switch (robot.Robot.armGardState)
                                {
                                    case OPEN:
                                    {
                                        RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos -= 0.001;
                                        Log.d(TAG_COLLECTION, "ARM_GARD_OPEN is " + RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos);
                                        robot.MoveServo.openGard();
                                        armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos;
                                    }
                                    break;
                                    case CLOSE:
                                    {
                                        RobotHardware_apollo.SERVO_POS.ARM_GARD_CLOSE.Pos -= 0.001;
                                        Log.d(TAG_COLLECTION, "ARM_GARD_CLOSE is " + RobotHardware_apollo.SERVO_POS.ARM_GARD_CLOSE.Pos);
                                        robot.MoveServo.closeGard();
                                        armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_GARD_CLOSE.Pos;
                                    }
                                    break;
                                    case HALF_OPEN:
                                    {
                                        RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos -= 0.001;
                                        Log.d(TAG_COLLECTION, "ARM_GARD_HALF_OPEN is " + RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos);
                                        robot.MoveServo.halfOpenGard();
                                        armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos;
                                    }
                                    break;
                                }
                            }
                            else
                            {
                                servoCurrentPosition = robot.GetPosServo.gard();
                                servoCurrentPosition -= 0.01;
                                robot.SetPosServo.gard(servoCurrentPosition);
                            }
                            pressCollectionConf = true;
                        }
                    }
                    else if (gamepad1.dpad_right)
                    {
                        if (!pressCollectionConf)
                        {
                            if (gamepad1.back)
                            {
                                switch (robot.Robot.armGardState)
                                {
                                    case OPEN: {
                                        RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos += 0.001;
                                        Log.d(TAG_COLLECTION, "ARM_GARD_OPEN is " + RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos);
                                        robot.MoveServo.openGard();
                                        armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_GARD_OPEN.Pos;
                                    }
                                    break;
                                    case CLOSE: {
                                        RobotHardware_apollo.SERVO_POS.ARM_GARD_CLOSE.Pos += 0.001;
                                        Log.d(TAG_COLLECTION, "ARM_GARD_CLOSE is " + RobotHardware_apollo.SERVO_POS.ARM_GARD_CLOSE.Pos);
                                        robot.MoveServo.closeGard();
                                        armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_GARD_CLOSE.Pos;
                                    }
                                    break;
                                    case HALF_OPEN:
                                    {
                                        RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos += 0.001;
                                        Log.d(TAG_COLLECTION, "ARM_GARD_HALF_OPEN is " + RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos);
                                        robot.MoveServo.halfOpenGard();
                                        armGardServoPos = RobotHardware_apollo.SERVO_POS.ARM_GARD_HALF_OPEN.Pos;
                                    }
                                    break;
                                }
                            }
                            else
                            {
                                servoCurrentPosition = robot.GetPosServo.gard();
                                servoCurrentPosition += 0.01;
                                robot.SetPosServo.gard(servoCurrentPosition);
                            }
                            pressCollectionConf = true;
                        }
                    }
                    else
                    {
                        pressCollectionConf = false;
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

                    else if (dumpPixel != 0)
                    {
                        if (pressCollectionServo == false)
                        {
                            pressCollectionServo = true;
                            DumpPixel();
                        }
                    }
                    else if(gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                    {
                        robot.MoveServo.openGard();
                    }
                    else
                    {
                        servoStayInPos();
                        pressCollectionServo = false;
                    }
                    if (pixelEmission != 0)
                    {
                        collectionMotor(collectionBackSpeed);
                        gamepad2.rumble(0.1,0.1,Gamepad.RUMBLE_DURATION_CONTINUOUS);
                    }
                    else if (pixelCollection == true)
                    {
                        if (pressCollection == false)
                        {
                            pressCollection = true;
                            CollectPixel();

                        }
                        collectionMotor(-collectionSpeed);
                        gamepad2.rumble(0.1,0.1,Gamepad.RUMBLE_DURATION_CONTINUOUS);
                    }
                    else
                    {
                        collectionMotor(0);
                        gamepad2.stopRumble();
                    }
                }
            }
            catch (Exception e)
            {
                Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
            }

        }
        public void lunchDrone()
        {
            if (robot.Robot.drone_state == RobotHardware_apollo.DRONE_STATE.LUNCH)
            {
                robot.MoveServo.loadDrone();
            }
            else if(robot.Robot.drone_state == RobotHardware_apollo.DRONE_STATE.LOADED)
            {
                robot.MoveServo.lunchDrone();
            }
        }
        public void servoStayInPos()
        {
            servoCurrentPosition = robot.GetPosServo.gard();
            robot.SetPosServo.gard(servoCurrentPosition);
        }
        public void collectionMotor (double speed)
        {
            robot.SetVelocity.collection(speed);
        }
        public void CollectPixel()
        {
            robot.MoveServo.collectPixel();
            robot.MoveServo.openGard();
        }
        public void DumpPixel(){
            if (robot.Robot.armState != RobotHardware_apollo.ArmState.COLLECT)
            {
                Log.d(TAG_COLLECTION,"armServoState is before " + robot.Robot.armState);
                Log.d(TAG_COLLECTION,"armServoGardState is before " + robot.Robot.armGardState);
                if(robot.Robot.armGardState == RobotHardware_apollo.ArmGardState.CLOSE)
                {
                    robot.MoveServo.halfOpenGard();
                    try
                    {
                        //Thread.sleep(400);
                    }  catch (Exception e)
                    {
                        Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                    }
                }
                else if(robot.Robot.armGardState == RobotHardware_apollo.ArmGardState.HALF_OPEN)
                {
                    robot.MoveServo.openGard();
                }
                else if (robot.Robot.armGardState == RobotHardware_apollo.ArmGardState.OPEN)
                {
                    robot.MoveServo.closeGard();
                }
                if (robot.GetPosServo.arm() != RobotHardware_apollo.SERVO_POS.ARM_DUMP.Pos)
                {
                    try
                    {
                        Thread.sleep(100);
                    }  catch (Exception e)
                    {
                        Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                    }
                    robot.MoveServo.dumpPixel();
                }

                Log.d(TAG_COLLECTION,"armServoState is next " + robot.Robot.armState);
                Log.d(TAG_COLLECTION,"armServoGardState is next " + robot.Robot.armGardState);
            }
            //}
        }
    }
    public class LiftThread extends Thread {
        private LiftState liftState;
        private ElapsedTime liftTime = new ElapsedTime();
        private double timerTimeoutInSeconds;

        public LiftThread() {
            this.setName("LiftThread");
            Log.d(TAG_COLLECTION_THREAD, "start Lift thread");
        }

        public void run() {
            liftState = LiftState.STOP;
            liftTime.reset();
            Log.d(TAG_LIFT, "start");
            while (!isInterrupted()) {
                boolean liftUp = gamepad2.dpad_up;
                boolean liftDown = gamepad2.dpad_down;
                boolean liftPositionY = gamepad2.y;
                boolean liftPositionA = gamepad2.a;
                boolean liftPositionB = gamepad2.b;
                boolean liftPositionX = gamepad2.x;
                boolean liftStop = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
                boolean liftReset = gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
                Log.d(TAG_LIFT, "liftPositionY " + liftPositionY);
                Log.d(TAG_LIFT, "liftPositionA " + liftPositionA);
                Log.d(TAG_LIFT, "liftPositionB " + liftPositionB);
                Log.d(TAG_LIFT, "liftPositionX " + liftPositionX);
                Log.d(TAG_LIFT, "lift pos is " + robot.GetPosMotor.lift());
                if ((robot.GetPosMotor.lift() < 500) || (runtime.seconds() > 3 * 60)) {
                    robot.Robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    robot.Robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.ZeroPowerBehavior.BRAKE);
                }
                if (liftStop) {
                    switch (robot.Robot.liftLockStat) {
                        case UNLOCKED:
                            robot.MoveServo.lockLift();
                            gamepad1.rumble(1, 1, 500);
                            Log.d(TAG_LIFT, "lift Pos at lock is " + robot.GetPosMotor.lift());
                            break;
                        case LOCKED:
                            robot.MoveServo.unlockLift();
                            break;
                        default:

                            break;
                    }
                }
                if (liftReset) {
                    liftState = LiftState.RESETING_INCODER;
                    liftPower = 0;
                    robot.SetPower.lift(liftPower);
                    robot.SetPower.secondLift(liftPower);
                    robot.SetMode.lift(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    telemetry.addLine("lift is at pos 0");
                }
                if (liftUp == false) {
                    resetIncoder();
                }
                if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY) {
                    if (isLiftBusy() == true) {
                        Log.d(TAG_LIFT, "run to position " + pos + " ; current position is " + robot.GetPosMotor.lift());
                    }
                    if (isLiftBusy() == false) {
                        EndOfGetToPos();
                    }
                }
                if (liftUp == true) {
                    if (robot.GetPosMotor.lift() < liftMaxHight) {
                        setGateServoToClose();
                        liftState = LiftState.MANUAL_CONTROL;
                        Log.d(TAG_LIFT, "lift up pos " + robot.GetPosMotor.lift());
                        robot.SetMode.lift(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftPower = POWER_LIFT;
                        robot.Robot.armState = RobotHardware_apollo.ArmState.DUMP;
                        inPosition = false;
                    } else {
                        if (inPosition == false) {
                            liftPower = 0;
                        }
                    }

                } else if (liftDown == true) {
                    if (liftState != LiftState.RESETING_INCODER) {
                        liftState = LiftState.MANUAL_CONTROL;
                        Log.d(TAG_LIFT, "lift down pos " + robot.GetPosMotor.lift());
                        robot.SetMode.lift(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftPower = -POWER_LIFT;
                        inPosition = false;
                    }
                } else if (liftPositionY == true) {
                    if (pressLift == false) {
                        pressLift = true;
                        try {
                            goToPosAndMoveArm(FIRST_LIFT);
                        } catch (Exception e) {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }

                    }

                } else if ((liftPositionB == true) && (!gamepad2.start) && (!gamepad1.start)) {
                    if (pressLift == false) {
                        pressLift = true;
                        try {
                            goToPosAndMoveArm(SECOND_LIFT);
                        } catch (Exception e) {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                    }

                } else if ((liftPositionA == true) && (!gamepadEx2.isDown(GamepadKeys.Button.START))) {
                    if (pressLift == false) {
                        pressLift = true;
                        try {
                            goToPosAndMoveArm(THIRD_LIFT);
                        } catch (Exception e) {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                    }

                } else if (liftPositionX == true) {
                    if (pressLift == false) {
                        pressLift = true;
                        try {
                            goToPosAndMoveArm(FOURTH_LIFT);
                        } catch (Exception e) {
                            Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                        }
                    }

                } else {
                    if ((inPosition == false) && (isLiftBusy() == false)) {
                        if (robot.GetPosMotor.lift() >= 0) {
                            stayInPos();
                        }

                    } else if (isLiftBusy() == false) {
                        liftState = LiftState.STOP;
                    }

                }
                if ((liftPositionA == false) && (liftPositionB == false) && (liftPositionX == false) && (liftPositionY == false) && (gamepad2.right_bumper == false) && (gamepad2.left_bumper == false) && (!liftDown) && (!liftUp)) {
                    pressLift = false;
                }

                if (liftPower != 0) {
                    Log.d(TAG_LIFT, "lift Power is " + liftPower);
                    Log.d(TAG_LIFT, "lift mode is  " + robot.Robot.GetMode(RobotHardware_apollo.DriveMotors.LIFT));
                    Log.d(TAG_LIFT, "current position " + robot.GetPosMotor.lift());

                }
                robot.SetPower.lift(liftPower);
                robot.SetPower.secondLift(liftPower);


            }
            Log.d(TAG_LIFT, "finish");

        }

        public void resetIncoder() {
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
            if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY) {
                if (isLiftBusy() == true) {
                    liftrest();
                }
            }
            if (Pos != 0) {
                setGateServoToClose();
            }
            if (Pos == 0) {
                try {
                    double currentPosition = robot.GetPosMotor.lift();
                    if (currentPosition < THIRD_LIFT) {
                        Thread.sleep(500);
                    }
                } catch (Exception e) {
                    Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
                }
            }
            telemetry.addData("run to position ", "%d", Pos);
            Log.d(TAG_LIFT, "run to position " + Pos);
            currentPosition = (int) robot.GetPosMotor.lift();
            pos = Pos;
            if (currentPosition > Pos) {
                liftPower = -POWER_LIFT;
            } else {
                liftPower = POWER_LIFT;
            }
            robot.SetPosMotor.lift(Pos, liftPower);
            robot.SetPower.secondLift(liftPower);
            inPosition = false;
            liftTime.reset();
            liftState = LiftState.AUTO_CONTROL_WAIT_FOR_BUSY;
            timerTimeoutInSeconds = LIFT_TIMEOUT_SEC;
        }

        private void stayInPos() {
            Log.d(TAG_LIFT, "stayInPos start");
            if (robot.GetPosMotor.lift() > 20) {
                robot.SetMode.lift(DcMotor.RunMode.RUN_USING_ENCODER);
                currentPosition = (int) robot.GetPosMotor.lift();
                ;
                currentPosition += 10;
                Log.d(TAG_LIFT, "stay in pos start, current Position is " + currentPosition);
                pos = (int) robot.GetPosMotor.lift();
                if (currentPosition > pos) {
                    liftPower = -POWER_LIFT;
                } else {
                    liftPower = POWER_LIFT;
                }
                robot.SetPosMotor.lift(currentPosition, liftPower);
                robot.SetPower.secondLift(liftPower);
                liftTime.reset();
                liftState = LiftState.AUTO_CONTROL_WAIT_FOR_BUSY;
                timerTimeoutInSeconds = LIFT_TIMEOUT_STAY_SEC;
            } else {
                robot.SetPower.lift(0);
                robot.SetPower.secondLift(0);
            }


        }

        private boolean isLiftBusy() {
            boolean isLiftBusy = false;
            if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY) {
                if ((robot.Robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT) == true) && (liftTime.seconds() < timerTimeoutInSeconds)) {
                    isLiftBusy = true;
                }
            }
            return (isLiftBusy);
        }

        private void EndOfGetToPos() {
            if (liftTime.seconds() > timerTimeoutInSeconds) {
                Log.d(TAG_LIFT, "stopped due to time out. stopped at " + robot.GetPosMotor.lift());
                liftState = LiftState.AUTO_CONTROL_ERROR;
                inPosition = true;

            } else {
                Log.d(TAG_LIFT, "Final Position is " + robot.GetPosMotor.lift());
                Log.d(TAG_LIFT, "timer in milliseconds is " + liftTime.milliseconds() + " timer in seconds is " + liftTime.seconds());
                inPosition = true;
                liftState = LiftState.AUTO_CONTROL_IN_POSITION;
            }
            robot.Robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.ZeroPowerBehavior.BRAKE);
        }
        private void liftrest() {
            stayInPos();
            Log.d(TAG_LIFT, "lift rest");
        }

        private void setGateServoToClose() {
            robot.MoveServo.closeGard();
        }
        private void goToPosAndMoveArm(int Pos) throws InterruptedException {
            robot.MoveServo.closeGard();
            goTo(Pos);
            if (Pos == FIRST_LIFT) {
                LIFT_IsBusy = robot.Robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
                TimeOut.reset();
                while ((LIFT_IsBusy) && (TimeOut.seconds() < TimeOutSec)) {
                    LIFT_IsBusy = robot.Robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
                }
            }
            robot.MoveServo.dumpPixel();
        }
    }
}
