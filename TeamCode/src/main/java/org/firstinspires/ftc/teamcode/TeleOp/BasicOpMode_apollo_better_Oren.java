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
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
@TeleOp(name="TeleOp apollo Oren", group="TeleOp")
//@Disabled
public class BasicOpMode_apollo_better_Oren extends OpMode {

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;
    //Gamepad.RumbleEffect liftLockRumble;


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime TimeOut = new ElapsedTime();
    private ElapsedTime CollectionAutoTimer = new ElapsedTime();
    private ElapsedTime PixelInDetectionTimeOut = new ElapsedTime();
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
    boolean pressAutoCollection = false;
    boolean LIFT_IsBusy;
    final int FIRST_LIFT = 662;
    double servoCurrentPosition;
    final int SECOND_LIFT = 860;
    final int THIRD_LIFT = 1630;
    final int FOURTH_LIFT = 2200;
    int liftMaxHight = 3420;
    final double POWER_LIFT = 1;
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
    double collectionLastPos = 0;
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
    int numOfPixelsToCollect = 0;
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
    int pixelInCollection = 0;
    int pixelNotInCollection = 0;
    boolean duringCollection = false;
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
        robot.Robot.collectionStats = RobotHardware_apollo.CollectionStats.NORMAL;
        robot.Robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        //telemetry.addLine("<h1> field Centric Drive stats is " + fieldCentricDrive + "</h1>");
        telemetry.addLine("<h1> heading is " + heading + "</h1>");
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
        telemetry.addData("<h1> num of Pixels is ", pixelInCollection + "</h1>");
        telemetry.addData("<h1> collection stat is  ", robot.Robot.collectionStats + "</h1>");
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
    private void drive() {
        gamepadEx1.readButtons();
        gamepadEx2.readButtons();

        double forwardSpeed;
        double turnSpeed;
        double strafeSpeed;
        forwardSpeed = gamepadEx1.getLeftY();  // Note: pushing stick forward gives negative value
        strafeSpeed = gamepadEx1.getLeftX();
        turnSpeed = -gamepadEx1.getRightX();
        heading = robot.Robot.getImuRawHeading();
        if ((gamepad1.share) || gamepad1.back){
            if (!pressDrive) {
                pressDrive = true;
                robot.Robot.ResetYaw();
                heading = robot.Robot.getImuRawHeading();
            }
        }
        else if (gamepad1.dpad_right) {
            if (!pressDrive) {
                pressDrive = true;
                controlMod = !controlMod;
            }
        } else {
            pressDrive = false;
        }
        /*
        if(gamepad1.a)
        {
            robot_Ftclib .driveRobotCentric(0,
                    -0.25,
                    0);
        }

         */
        if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            desiredHeading = 45;
            holdHeading(TURN_SPEED, desiredHeading);
        } else if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            desiredHeading = 90;
            holdHeading(TURN_SPEED, desiredHeading);
        } else {
            if (controlMod) {
                robot_Ftclib.driveFieldCentric(
                        strafeSpeed / 2,
                        forwardSpeed / 3,
                        turnSpeed / 3,
                        heading

                );
            } else {
                robot_Ftclib.driveFieldCentric(
                        strafeSpeed,
                        forwardSpeed,
                        turnSpeed,
                        heading
                );
            }
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
        robot_Ftclib.SetPower(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, -backLeftPower);
        robot_Ftclib.SetPower(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightPower);
        robot_Ftclib.SetPower(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightPower);
        robot_Ftclib.SetPower(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, -frontLeftPower);
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
                    double pixelCollection = gamepad1.left_trigger;
                    double pixelEmission = gamepad1.right_trigger;
                    boolean collectPixel = gamepad1.left_bumper;
                    boolean dumpPixel =  gamepad1.right_bumper;
                    //detectPixel();
                    if ((gamepad1.options) || gamepad1.start)
                    {
                        if (pressDrone == false)
                        {
                            pressDrone = true;
                            lunchDrone();
                            gamepad1.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                        }
                    }
                    else
                    {
                        pressDrone = false;
                    }
                    switch (robot.Robot.collectionStats)
                    {
                        case FINISHED_AUTO:
                        case NORMAL:
                            if (gamepad1.ps)
                            {
                                if (!pressAutoCollection)
                                {
                                    pressAutoCollection = true;
                                    numOfPixelsToCollect = 2;
                                    pixelInCollection = 0;
                                    collectionLastPos = 0;
                                    robot.Robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    robot.Robot.SetMode(RobotHardware_apollo.DriveMotors.COLLECTION, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    robot.SetPower.collection(-1);
                                    CollectionAutoTimer.reset();
                                    robot.Robot.collectionStats = RobotHardware_apollo.CollectionStats.AUTO;
                                    gamepad1.rumbleBlips(3);
                                    break;
                                }
                            }
                            else if (pixelEmission != 0)
                            {
                                robot.SetPower.collection(pixelEmission);
                                //collectionMotor(collectionBackSpeed);
                                robot.Robot.collectionStats = RobotHardware_apollo.CollectionStats.NORMAL;
                                gamepad1.rumble(0.1,0.1,Gamepad.RUMBLE_DURATION_CONTINUOUS);
                            }
                            else if (pixelCollection != 0)
                            {
                                if (pressCollection == false)
                                {
                                    pressCollection = true;
                                    CollectPixel();

                                }
                                collectionMotor(-collectionSpeed);
                                robot.Robot.collectionStats = RobotHardware_apollo.CollectionStats.NORMAL;
                                gamepad1.rumble(0.1,0.1,Gamepad.RUMBLE_DURATION_CONTINUOUS);
                            }
                            else
                            {
                                if (robot.Robot.collectionStats == RobotHardware_apollo.CollectionStats.NORMAL)
                                {
                                    collectionMotor(0);
                                }
                                if((!pressDrone) && (robot.Robot.collectionStats == RobotHardware_apollo.CollectionStats.NORMAL))
                                {
                                    gamepad1.stopRumble();
                                }
                                pressAutoCollection = false;
                            }
                            break;
                        case AUTO:
                            detectPixel();
                            if (gamepad1.ps)
                            {
                                if (!pressAutoCollection)
                                {
                                    pressAutoCollection = true;
                                    gamepad1.rumbleBlips(2);
                                    robot.Robot.collectionStats = RobotHardware_apollo.CollectionStats.NORMAL;
                                }
                            }
                            else
                            {
                                pressAutoCollection = false;
                            }
                            double collectionPos = robot.GetPosMotor.collection();
                            Log.d(TAG_COLLECTION,"collection pos is " + collectionPos);
                            if (robot.Robot.GetPower(RobotHardware_apollo.DriveMotors.COLLECTION) < 0)
                            {
                                if (collectionPos + 30 > collectionLastPos)
                                {
                                    robot.SetPower.collection(1);
                                }
                            }
                            else if (robot.Robot.GetPower(RobotHardware_apollo.DriveMotors.COLLECTION) > 0)
                            {
                                if (collectionPos - 50 > collectionLastPos)
                                {
                                    robot.SetPower.collection(-1);
                                    Log.d(TAG_COLLECTION,"detected jum a pos " + collectionPos);
                                    collectionLastPos = collectionPos;
                                }
                            }
                            else if (pixelInCollection != numOfPixelsToCollect)
                            {
                                robot.SetPower.collection(-1);
                            }
                            if (!duringCollection && pixelInCollection == numOfPixelsToCollect)
                            {
                                robot.SetPower.collection(1);
                                robot.Robot.collectionStats = RobotHardware_apollo.CollectionStats.FINISHED_AUTO;
                                gamepad1.rumbleBlips(3);
                                break;
                            }
                            if (duringCollection)
                            {
                                if (PixelInDetectionTimeOut.seconds() >= 2)
                                {
                                    pixelInCollection--;
                                    robot.SetPower.collection(1);
                                    PixelInDetectionTimeOut.reset();
                                    sleep(250);
                                }
                            }
                            else
                            {
                                PixelInDetectionTimeOut.reset();
                            }
                            if (CollectionAutoTimer.seconds() > 1)
                            {
                                Log.d(TAG_COLLECTION,"counted collection pos is " + collectionPos);
                                collectionLastPos = robot.GetPosMotor.collection();
                                CollectionAutoTimer.reset();
                            }
                            break;
                    }
                    if (collectPixel == true)
                    {
                        if (pressCollectionServo == false)
                        {
                            pressCollectionServo = true;
                            if (robot.Robot.armState == RobotHardware_apollo.ArmState.HIGH_COLLECT)
                            {
                                CollectPixel();
                                liftTread.goTo(0);
                            }
                            else
                            {
                                robot.Robot.armState = RobotHardware_apollo.ArmState.HIGH_COLLECT;
                                robot.SetPosServo.arm(RobotHardware_apollo.SERVO_POS.ARM_DUMP_AUTO.Pos);
                            }
                        }
                    }
                    else if (dumpPixel == true)
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
                }
            }
            catch (Exception e)
            {
                Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
            }

        }
        public void detectPixel()
        {
            double distance = robot.Robot.getDistance(DistanceUnit.CM);
            if (distance < 8)
            {
                pixelNotInCollection = 0;
                if (!duringCollection)
                {
                    duringCollection = true;
                    if (pixelInCollection <= 100)
                    {
                        pixelInCollection++;
                    }
                }
            }
            else
            {
                if (pixelNotInCollection >= 7)
                {
                    duringCollection = false;
                }
                else
                {
                    pixelNotInCollection++;
                }
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
                boolean liftUp = gamepad1.dpad_up;
                boolean liftDown = gamepad1.dpad_down;
                boolean liftPositionY = gamepad1.y;
                boolean liftPositionA = gamepad1.a;
                boolean liftPositionB = gamepad1.b;
                boolean liftPositionX = gamepad1.x;
                //boolean liftStop = gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
                //boolean liftReset = gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
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
                /*
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

                 */
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
                if ((liftPositionA == false) && (liftPositionB == false) && (liftPositionX == false) && (liftPositionY == false) && (gamepad1.right_bumper == false) && (gamepad1.left_bumper == false) && (!liftDown) && (!liftUp)) {
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
