/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp(name = "Sensor: HuskyLens conf Apollo", group = "UnitTest")
//@Disabled
public class SensorHuskyLens_Apollo_conf extends OpMode {

    private final int READ_PERIOD = 1;

    final String TAG_HUSKYLENS = "HuskyLens_Apollo";
    private double Offset = 35;
    public static double UpX_Max = 182;
    public static double UpX_Min = 185;
    public static double UpY_Max = 105;
    public static double UpY_Min = 130;
    public static double RightX_Max = 71;
    public static double RightX_Min = 125;
    public static double RightY_Max = 100;
    public static double RightY_Min = 100;
    private HuskyLens huskyLens;
    private boolean isPress = false;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        huskyLensState = HuskyLens_State.COLOR_RECOGNITION;


        telemetry.update();
    }

    @Override
    public void loop() {
        blocks = huskyLens.blocks();
        if(gamepad1.a == true)
        {
            if (isPress = false)
            {
                isPress = true;
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
                huskyLensState = HuskyLens_State.COLOR_RECOGNITION;
            }
        }
        else if(gamepad1.b == true)
        {
            if (isPress = false)
            {
                isPress = true;
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                huskyLensState = HuskyLens_State.COLOR_RECOGNITION;
            }
        }
        if (gamepad1.a == false && gamepad1.b == false)
        {
            isPress = false;
        }
        if (huskyLensState == HuskyLens_State.COLOR_RECOGNITION)
        {
            COLOR_RECOGNITION();
        }
        if (huskyLensState == HuskyLens_State.TAG_RECOGNITION)
        {
            TAG_RECOGNITION();
        }


        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
        }
    }

    private enum HuskyLens_State {TAG_RECOGNITION,
                 COLOR_RECOGNITION};
    HuskyLens.Block[] blocks;

    private HuskyLens_State huskyLensState;
    private void TAG_RECOGNITION()
    {

    }
    private void COLOR_RECOGNITION()
    {
        blocks = huskyLens.blocks();
        if(blocks.length != 0)
        {
            for (int i = 0; i < blocks.length; i++)
            {
                if (((UpX_Min + Offset <= blocks[i].x) || (UpX_Min - Offset <= blocks[i].x)) && ((blocks[i].x <= UpX_Max + Offset) || (blocks[i].x <= UpX_Max - Offset)) && ((blocks[i].y >= UpY_Min + Offset) || blocks[i].y >= UpY_Min - Offset)  && (blocks[i].y <= UpY_Max + Offset) || (blocks[i].y <= UpY_Max - Offset))
                {
                    telemetry.addLine("The Prop is on line Up");
                    Log.d(TAG_HUSKYLENS, "The Prop is on line Up");
                }
                else if (((RightX_Min + Offset <= blocks[i].x) || (RightX_Min - Offset<= blocks[i].x)) && ((blocks[i].x <= RightX_Max + Offset) || (blocks[i].x <= RightX_Max - Offset)) && ((blocks[i].y >= RightY_Min + Offset) || blocks[i].y >= RightY_Min - Offset)  && (blocks[i].y <= RightY_Max + Offset) || (blocks[i].y <= RightY_Max - Offset))
                {
                    telemetry.addLine("The Prop is on line right");
                    Log.d(TAG_HUSKYLENS, "The Prop is on line right");
                }
            }

        }
        telemetry.update();
    }
}