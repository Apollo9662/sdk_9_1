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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

public class RobotMoveServo extends RobotHardware_apollo{
    public void openDumpServo() {SetPosition(RobotHardware_apollo.DriveMotors.DUMP_SERVO, RobotHardware_apollo.SERVO_POS.DUMP_SERVO_OPEN.Pos);}
    public void closeDumpServo() {SetPosition(RobotHardware_apollo.DriveMotors.DUMP_SERVO, RobotHardware_apollo.SERVO_POS.DUMP_SERVO_CLOSE.Pos);}
    public void lunchDrone() {SetPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO, RobotHardware_apollo.SERVO_POS.PLANE_SERVO_OPEN.Pos);}
    public void loadDrone() {SetPosition(RobotHardware_apollo.DriveMotors.PLANE_SERVO, RobotHardware_apollo.SERVO_POS.PLANE_SERVO_CLOSE.Pos);}
    public void lockLift() {SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_CLOSE.Pos);}
    public void unlockLift() {SetPosition(RobotHardware_apollo.DriveMotors.LIFT_STOP_SERVO, RobotHardware_apollo.SERVO_POS.LIFT_STOP_SERVO_OPEN.Pos);}
    public void OpenGard() {SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_POS.Pos);}
    public void halfOpenGard() {SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_OPEN_CLOSE_POS.Pos);}
    public void closeOpenGard() {SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_GARD_CLOSE_POS.Pos);}
    public void dumpPixel() {SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_DUMP_POS.Pos);}
    public void collectPixel() {SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, RobotHardware_apollo.SERVO_POS.ARM_SERVO_COLLECT_POS.Pos);}
}