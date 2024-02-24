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

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    private MecanumDrive mecanumDriveBase;
    enum ArmServoGardState{OPEN,
        CLOSE,
        OPEN_CLOSE}
    ArmServoGardState armServoGardState;
    enum ArmServoState{COLLECT,
        DUMP}
    ArmServoState armServoState;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    final String TAG_HARDWARE = "HardwareApollo";
    private HuskyLens huskyLens;
    private BNO055IMU imu = null;
    private TouchSensor touchSensor1 = null;
    private TouchSensor touchSensor2 = null;
    private Servo armServo = null;
    private Servo dumpServo = null;
    private Servo planeServo = null;
    private Servo armGardServo = null;
    private Servo liftStopServo = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx collection = null;
    public DcMotorEx lift = null;
    private DcMotorEx liftSecond = null;  // private     public DcMotorEx lift = null; // private

    public static DriveMotors driveMotors;
    public enum DriveMotors {BACK_LEFT_DRIVE,
            FRONT_LEFT_DRIVE,
            FRONT_RIGHT_DRIVE,
            BACK_RIGHT_DRIVE,
            ARM_SERVO,
            ARM_GARD_SERVO,
            PLANE_SERVO,
            DUMP_SERVO,
            TOUCH_SENSOR1,
            TOUCH_SENSOR2,
            LIFT,
            LIFT_SECOND,
            LIFT_STOP_SERVO,
            COLLECTION};
    public enum PLANE_STATE
    {
        OPEN,
        CLOSE
    };
    PLANE_STATE plane_state;
    public enum SERVO_POS {
        DUMP_SERVO_CLOSE (0.9),
        LIFT_STOP_SERVO_OPEN(0.85),
        LIFT_STOP_SERVO_CLOSE(0.10),
        DUMP_SERVO_OPEN (0.32),
        PLANE_SERVO_CLOSE (0.9),
        PLANE_SERVO_OPEN (0.25),
        ARM_SERVO_COLLECT_POS (1.0),
        ARM_SERVO_DUMP_POS (0.4594),
        ARM_SERVO_DUMP_POS_AUTO_DRIVE (0.44),
        ARM_SERVO_GARD_OPEN_POS (0.3),
        ARM_SERVO_GARD_CLOSE_POS (0.61),
        ARM_SERVO_GARD_OPEN_CLOSE_POS (0.485);

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
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
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
        armServoState = ArmServoState.COLLECT;
        armServoGardState = ArmServoGardState.OPEN;
        armServo.setPosition(SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);
        armGardServo.setPosition(SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);
        dumpServo.setPosition(SERVO_POS.DUMP_SERVO_CLOSE.Pos);
        plane_state = PLANE_STATE.CLOSE;
        planeServo.setPosition(SERVO_POS.PLANE_SERVO_CLOSE.Pos);
        liftStopServo.setPosition(SERVO_POS.LIFT_STOP_SERVO_OPEN.Pos);

    }
    public void ImuInit()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
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
                liftSecond.setPower(Power);
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
                liftSecond.setVelocity(Power);
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
            case PLANE_SERVO:
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

            case PLANE_SERVO:
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
                //liftSecond.setZeroPowerBehavior(myZeroPowerBehavior);
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
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


}


