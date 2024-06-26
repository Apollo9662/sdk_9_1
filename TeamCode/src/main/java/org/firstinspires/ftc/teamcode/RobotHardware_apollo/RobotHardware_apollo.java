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

package org.firstinspires.ftc.teamcode.RobotHardware_apollo;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class RobotHardware_apollo {

    public enum ArmGardState{OPEN,
        CLOSE,
        HALF_OPEN}
    public ArmGardState armGardState;
    public enum ArmState{COLLECT,
        HIGH_COLLECT,
        DUMP}
    public enum LiftLockStat
    {
        LOCKED,
        UNLOCKED
    }
    public LiftLockStat liftLockStat;
    public ArmState armState;
    public double headingOffset = 0;
    /* Declare OpMode members. */
    private HardwareMap myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    final String TAG_HARDWARE = "HardwareApollo";
    private HuskyLens huskyLens;
    private BNO055IMU imu = null;
    private TouchSensor touchSensor1 = null;
    private TouchSensor touchSensor2 = null;
    private DistanceSensor sensorDistance = null;
    private Servo armServo = null;
    private Servo dumpServo = null;
    private Servo planeServo = null;
    private Servo armGardServo = null;
    private Servo liftStopServo = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    public DcMotorEx collection = null;
    private DcMotorEx lift = null;
    private DcMotorEx liftSecond = null;  // private     public DcMotorEx lift = null; // private

    public static DriveMotors driveMotors;
    public enum DriveMotors {BACK_LEFT_DRIVE,
            FRONT_LEFT_DRIVE,
            FRONT_RIGHT_DRIVE,
            BACK_RIGHT_DRIVE,
            ARM_SERVO,
            ARM_GARD_SERVO,
            DRONE_SERVO,
            DUMP_SERVO,
            TOUCH_SENSOR1,
            LIFT,
            LIFT_SECOND,
            LIFT_STOP_SERVO,
            COLLECTION}
    public enum DRONE_STATE
    {
        LUNCH,
        LOADED
    }
    public enum CollectionStats
    {
        NORMAL,
        AUTO,
        FINISHED_AUTO
    }
    public CollectionStats collectionStats;
    public DRONE_STATE drone_state;
    public enum SERVO_POS {
        //DUMP_SERVO_CLOSE (0.9),
        DUMP_LOAD_PIXEL (0.89),
        DUMP_UNLOAD_PIXEL (0.32),
        LIFT_UNLOCK(1.0),
        LIFT_LOCK(0.89),
        DRONE_LOAD (0.9),
        DRONE_LUNCH (0.25),
        ARM_COLLECT (1.0),
        ARM_DUMP (0.4389),
        ARM_DUMP_AUTO (0.19),
        ARM_GARD_OPEN (0.46),
        ARM_GARD_CLOSE (0.5),
        ARM_GARD_HALF_OPEN (0.475);

        public Double Pos;

        private SERVO_POS(Double Pos) {
            this.Pos = Pos;
        }
    }
    //public SERVO_POS servo_pos;
    /*
    public double ARM_SERVO_COLLECT_POS = 0.2;
    public double ARM_SERVO_DUMP_POS = 0.8;
    public double ARM_SERVO_GARD_OPEN_POS = 0;
    public double ARM_SERVO_GARD_CLOSE_POS = 0.32;
    public double ARM_SERVO_GARD_OPEN_CLOSE_POS = 0.13;

     */
    // Define a constructor that allows the OpMode to pass a reference to itself.
    //public RobotHardware_apollo(LinearOpMode opmode) {
        //myOpMode = opmode;
    //}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public boolean init(HardwareMap apolloHardwareMap, boolean initDrive , boolean initImu)
    {
        myOpMode = apolloHardwareMap;
        boolean intSucceeded = true;
        if (initDrive)
        {
            backLeftDrive = apolloHardwareMap.get(DcMotorEx.class, "back_left_drive"); //0
            frontLeftDrive = apolloHardwareMap.get(DcMotorEx.class, "front_left_drive"); //3
            backRightDrive = apolloHardwareMap.get(DcMotorEx.class, "back_right_drive"); //2
            frontRightDrive = apolloHardwareMap.get(DcMotorEx.class, "front_right_drive");//3
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (initImu)
        {
            imu = apolloHardwareMap.get(BNO055IMU.class, "imu2");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            boolean imuInitialize = imu.initialize(parameters);
            if (imuInitialize == false)
            {
                imu = apolloHardwareMap.get(BNO055IMU.class, "imu1");
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                imuInitialize = imu.initialize(parameters);
                if (imuInitialize == false)
                {
                    Log.d(TAG_HARDWARE, "initialization of imu1 failed");
                    intSucceeded = false;
                }
                else
                {
                    Log.d(TAG_HARDWARE, "initialization of imu1 succeeded");
                }
            }
            else
            {
                Log.d(TAG_HARDWARE, "initialization of imu2 succeeded");
            }


        }
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        collection = apolloHardwareMap.get(DcMotorEx.class, "collection");//0
        lift = apolloHardwareMap.get(DcMotorEx.class, "lift");//2
        liftSecond = apolloHardwareMap.get(DcMotorEx.class, "second_lift");
        //touchSensor1 = apolloHardwareMap.get(TouchSensor.class, "sensor_touch1");
        touchSensor2 = apolloHardwareMap.get(TouchSensor.class, "sensor_touch");
        sensorDistance = apolloHardwareMap.get(DistanceSensor.class, "sensor_distance");
        armServo = apolloHardwareMap.get(Servo.class, "arm_servo");//0
        liftStopServo = apolloHardwareMap.get(Servo.class, "lift_stop_servo");//5
        planeServo = apolloHardwareMap.get(Servo.class, "plane_servo");//
        dumpServo = apolloHardwareMap.get(Servo.class, "dump_servo");//3
        armGardServo = apolloHardwareMap.get(Servo.class, "arm_gard_servo");//2
        huskyLens = apolloHardwareMap.get(HuskyLens.class, "huskylens");
        collection.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        liftSecond.setDirection(DcMotorSimple.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
        dumpServo.setDirection(Servo.Direction.REVERSE);
        armGardServo.setDirection(Servo.Direction.FORWARD);
        collection.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collection.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collection.setVelocityPIDFCoefficients(0,0,0,12.8);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftSecond.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return intSucceeded;
    }
    public void SetAllMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior myZeroPowerBehavior)
    {
        frontLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        frontRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        backRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        backLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
    }
    public void ServoInit()
    {
        armState = ArmState.COLLECT;
        armGardState = ArmGardState.OPEN;
        armServo.setPosition(SERVO_POS.ARM_COLLECT.Pos);
        armGardServo.setPosition(SERVO_POS.ARM_GARD_OPEN.Pos);
        dumpServo.setPosition(SERVO_POS.DUMP_LOAD_PIXEL.Pos);
        drone_state = DRONE_STATE.LOADED;
        planeServo.setPosition(SERVO_POS.DRONE_LOAD.Pos);
        liftLockStat = LiftLockStat.UNLOCKED;
        liftStopServo.setPosition(SERVO_POS.LIFT_UNLOCK.Pos);

    }
    public HuskyLens getHuskyLens() {
        return huskyLens;
    }
    public void SetPower(DriveMotors motor, double Power)
    {
        switch (motor)
        {

            case BACK_LEFT_DRIVE:
            {
                backLeftDrive.setPower(Power);
            }
            break;
            case BACK_RIGHT_DRIVE:
            {
                backRightDrive.setPower(Power);
            }
            break;
            case FRONT_LEFT_DRIVE:
            {
                frontLeftDrive.setPower(Power);
            }
            break;
            case FRONT_RIGHT_DRIVE:
            {
                frontRightDrive.setPower(Power);
            }
            break;
            case LIFT:
            {
                lift.setPower(Power);
                //liftSecond.setPower(Power);
            }
            break;
            case LIFT_SECOND:
            {
                liftSecond.setPower(Power);
            }
            break;
            case COLLECTION:
            {
                collection.setPower(Power);
            }

            break;
            default:
                break;
        }
    }
    public void SetVelocity(DriveMotors motor, double Power)
    {
        switch (motor)
        {

            case BACK_LEFT_DRIVE:
            {
                backLeftDrive.setVelocity(Power);
            }
            break;
            case BACK_RIGHT_DRIVE:
            {
                backRightDrive.setVelocity(Power);
            }
            break;
            case FRONT_LEFT_DRIVE:
            {
                frontLeftDrive.setVelocity(Power);
            }
            break;
            case FRONT_RIGHT_DRIVE:
            {
                frontRightDrive.setVelocity(Power);
            }
            break;
            case LIFT:
            {
                lift.setVelocity(Power);
                //liftSecond.setVelocity(Power);
            }
            break;
            case LIFT_SECOND:
            {
                liftSecond.setVelocity(Power);
            }
            break;
            case COLLECTION:
            {
                collection.setVelocity(Power);
            }

            break;
            default:
                break;
        }
    }
    public double GetPower(DriveMotors motor)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE:
            {
                return backLeftDrive.getPower();
            }
            case BACK_RIGHT_DRIVE:
            {
                return backRightDrive.getPower();
            }
            case FRONT_LEFT_DRIVE:
            {
                return frontLeftDrive.getPower();
            }
            case FRONT_RIGHT_DRIVE:
            {
                return frontRightDrive.getPower();
            }
            case LIFT:
            {
                return lift.getPower();
            }
            case LIFT_SECOND:
            {
                return liftSecond.getPower();
            }
            case COLLECTION:
            {
                return collection.getPower();
            }
            default:
                return (0);
        }
    }
    public double GetCurrent(DriveMotors motor)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE:
            {
                return backLeftDrive.getCurrent(CurrentUnit.AMPS);
            }
            case BACK_RIGHT_DRIVE:
            {
                return backRightDrive.getCurrent(CurrentUnit.AMPS);
            }
            case FRONT_LEFT_DRIVE:
            {
                return frontLeftDrive.getCurrent(CurrentUnit.AMPS);
            }
            case FRONT_RIGHT_DRIVE:
            {
                return frontRightDrive.getCurrent(CurrentUnit.AMPS);
            }
            case LIFT:
            {
                return lift.getCurrent(CurrentUnit.AMPS);
            }
            case LIFT_SECOND:
            {
                return liftSecond.getCurrent(CurrentUnit.AMPS);
            }
            case COLLECTION:
            {
                return collection.getCurrent(CurrentUnit.AMPS);
            }
            default:
                return (0);
        }
    }
    public DcMotor.RunMode GetMode(DriveMotors motor)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE:
            {
                return backLeftDrive.getMode();
            }
            case BACK_RIGHT_DRIVE:
            {
                return backRightDrive.getMode();
            }
            case FRONT_LEFT_DRIVE:
            {
                return frontLeftDrive.getMode();
            }
            case FRONT_RIGHT_DRIVE:
            {
                return frontRightDrive.getMode();
            }
            case LIFT:
            {
                return lift.getMode();
            }
            case LIFT_SECOND:
            {
                return liftSecond.getMode();
            }
            case COLLECTION:
            {
                return collection.getMode();
            }
            default:
                return (null);
        }
    }
    public DcMotor.ZeroPowerBehavior GetZeroPowerBehavior(DriveMotors motor)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE:
            {
                return backLeftDrive.getZeroPowerBehavior();
            }
            case BACK_RIGHT_DRIVE:
            {
                return backRightDrive.getZeroPowerBehavior();
            }
            case FRONT_LEFT_DRIVE:
            {
                return frontLeftDrive.getZeroPowerBehavior();
            }
            case FRONT_RIGHT_DRIVE:
            {
                return frontRightDrive.getZeroPowerBehavior();
            }
            case LIFT:
            {
                return lift.getZeroPowerBehavior();
            }
            case LIFT_SECOND:
            {
                return liftSecond.getZeroPowerBehavior();
            }
            case COLLECTION:
            {
                return collection.getZeroPowerBehavior();
            }
            default:
                return (null);
        }
    }
    public PIDFCoefficients GetPIDFCoefficients(DriveMotors motor, DcMotor.RunMode myMode)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE:
            {
                return backLeftDrive.getPIDFCoefficients(myMode);
            }
            case BACK_RIGHT_DRIVE:
            {
                return backRightDrive.getPIDFCoefficients(myMode);
            }
            case FRONT_LEFT_DRIVE:
            {
                return frontLeftDrive.getPIDFCoefficients(myMode);
            }
            case FRONT_RIGHT_DRIVE:
            {
                return frontRightDrive.getPIDFCoefficients(myMode);
            }
            case LIFT:
            {
                return lift.getPIDFCoefficients(myMode);
            }
            case LIFT_SECOND:
            {
                return liftSecond.getPIDFCoefficients(myMode);
            }
            case COLLECTION:
            {
                return collection.getPIDFCoefficients(myMode);
            }
            default:
                return (null);
        }
    }
    public double GetVelocity(DriveMotors motor)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE:
            {
                return backLeftDrive.getVelocity();
            }
            case BACK_RIGHT_DRIVE:
            {
                return backRightDrive.getVelocity();
            }
            case FRONT_LEFT_DRIVE:
            {
                return frontLeftDrive.getVelocity();
            }
            case FRONT_RIGHT_DRIVE:
            {
                return frontRightDrive.getVelocity();
            }
            case LIFT:
            {
                return lift.getVelocity();
            }
            case LIFT_SECOND:
            {
                return liftSecond.getVelocity();
            }
            case COLLECTION:
            {
                return collection.getVelocity();
            }
            default:
                return (0);
        }
    }
    public double GetCurrentPosition(DriveMotors motor)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE: {
                return (backLeftDrive.getCurrentPosition());
            }
            case BACK_RIGHT_DRIVE: {
                return (backRightDrive.getCurrentPosition());
            }
            case FRONT_LEFT_DRIVE: {
                return (frontLeftDrive.getCurrentPosition());
            }
            case FRONT_RIGHT_DRIVE: {
                return (frontRightDrive.getCurrentPosition());
            }
            case COLLECTION:
            {
                return (collection.getCurrentPosition());
            }
            case LIFT:
            {
                return (lift.getCurrentPosition());
            }
            case LIFT_SECOND:
            {
                return (liftSecond.getCurrentPosition());
            }
            case ARM_SERVO:
            {
                return (armServo.getPosition());
            }
            case ARM_GARD_SERVO:
            {
                return (armGardServo.getPosition());
            }
            case DRONE_SERVO:
            {
                return (planeServo.getPosition());
            }
            case DUMP_SERVO:
            {
                return (dumpServo.getPosition());
            }
            case LIFT_STOP_SERVO:
            {
                return (liftStopServo.getPosition());
            }
            default:
                return (1);
        }
    }
        public void SetTargetPosition(DriveMotors motor, int Position)
        {
            switch (motor) {

                case BACK_LEFT_DRIVE: {
                    backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backLeftDrive.setTargetPosition(Position);
                }
                break;
                case BACK_RIGHT_DRIVE: {
                    backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backRightDrive.setTargetPosition(Position);
                }
                break;
                case FRONT_LEFT_DRIVE: {
                    frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontLeftDrive.setTargetPosition(Position);
                }
                break;
                case FRONT_RIGHT_DRIVE: {
                    frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRightDrive.setTargetPosition(Position);
                }
                break;
                case COLLECTION:{
                    collection.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    collection.setTargetPosition(Position);
                }
                case LIFT: {
                    //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setTargetPosition(Position);
                    //liftSecond.setTargetPosition(Position);
                }
                case LIFT_SECOND: {
                    liftSecond.setTargetPosition(Position);
                }

                break;
                default:
                    break;
            }
        }
    public void SetMode(DriveMotors motor, DcMotor.RunMode myMode)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE: {
                backLeftDrive.setMode(myMode);
            }
            break;
            case BACK_RIGHT_DRIVE: {
                backRightDrive.setMode(myMode);
            }
            break;
            case FRONT_LEFT_DRIVE: {
                frontLeftDrive.setMode(myMode);
            }
            break;
            case FRONT_RIGHT_DRIVE: {
                frontRightDrive.setMode(myMode);
            }
            break;
            case COLLECTION:
            {
                collection.setMode(myMode);
            }
            case LIFT: {
                lift.setMode(myMode);
                //liftSecond.setMode(myMode);
            }
            case LIFT_SECOND: {
                liftSecond.setMode(myMode);
            }
            break;
            default:
                break;
        }
    }
    public void SetAllDriveMotorsMode(DcMotor.RunMode myMode)
    {
        frontLeftDrive.setMode(myMode);
        frontRightDrive.setMode(myMode);
        backRightDrive.setMode(myMode);
        backLeftDrive.setMode(myMode);
    }
    public void SetPosition(DriveMotors motor, double Position)
    {
        switch (motor) {
            case DRONE_SERVO:
                planeServo.setPosition(Position);
            break;
            case ARM_SERVO:
                armServo.setPosition(Position);
            break;
            case ARM_GARD_SERVO:
                armGardServo.setPosition(Position);
            break;
            case DUMP_SERVO:
                dumpServo.setPosition(Position);
            break;
            case LIFT_STOP_SERVO:
                liftStopServo.setPosition(Position);
            default:
                break;
        }
    }
    public boolean IsPressed(DriveMotors motor)
    {
        switch (motor)
        {
            case TOUCH_SENSOR1:
                return touchSensor1.isPressed();
            case BACK_RIGHT_DRIVE:
                return touchSensor2.isPressed();
            default:
                return (false);
        }
    }
    public boolean IsBusy(DriveMotors motor)
    {
        switch (motor)
        {
            case BACK_LEFT_DRIVE: {
                return backLeftDrive.isBusy();
            }
            case BACK_RIGHT_DRIVE: {
                return backRightDrive.isBusy();
            }
            case FRONT_LEFT_DRIVE: {
                return frontLeftDrive.isBusy();
            }
            case FRONT_RIGHT_DRIVE: {
                return frontRightDrive.isBusy();
            }
            case LIFT:
                return lift.isBusy();
            case LIFT_SECOND:
                return liftSecond.isBusy();
            default:
                return (false);
        }
    }
    public void SetZeroPowerBehavior(DriveMotors motor, DcMotor.ZeroPowerBehavior myZeroPowerBehavior)
    {
        switch (motor) {

            case BACK_LEFT_DRIVE: {
                backLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
            }
            break;
            case BACK_RIGHT_DRIVE: {
                backRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
            }
            break;
            case FRONT_LEFT_DRIVE: {
                frontLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
            }
            break;
            case FRONT_RIGHT_DRIVE: {
                frontRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
            }
            case LIFT:
                lift.setZeroPowerBehavior(myZeroPowerBehavior);
            case LIFT_SECOND:
                liftSecond.setZeroPowerBehavior(myZeroPowerBehavior);
            break;
            default:
                break;
        }
    }
    public void SetAllDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior myZeroPowerBehavior)
    {
        frontLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        frontRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        backRightDrive.setZeroPowerBehavior(myZeroPowerBehavior);
        backLeftDrive.setZeroPowerBehavior(myZeroPowerBehavior);
    }
    public double getImuRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles.firstAngle -= headingOffset;
        return angles.firstAngle;
    }
    public void ResetYaw()
    {
        headingOffset += getImuRawHeading();
    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : myOpMode.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public double getDistance(DistanceUnit DistanceUnit)
    {
        return sensorDistance.getDistance(DistanceUnit);
    }
}



