package org.firstinspires.ftc.teamcode.AutoDrive;

import android.util.Log;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class HuskyLens_Apollo
{
    final String TAG_HUSKYLENS = "HuskyLens_Apollo";
    //private final int READ_PERIOD = 1;
    public int middle = 185;
    public int maxTop = 60;
    public int minTop = 25;
    public int maxWidth = 60;
    public int maxHeight = 60;
    public int numOfBlocks;
    private boolean isPress = false;
    private enum HuskyLens_State {TAG_RECOGNITION,
        COLOR_RECOGNITION};
    public enum PropPos{UP,
                        RIGHT,
                        LEFT};
    public enum PropColor {RED,BLUE};
    HuskyLens.Block[] blocks;

    private HuskyLens_State huskyLensState;
    private HuskyLens huskyLens_apollo;
    private PropColor propColor;
    private int propId;
    private int propId2 = 0;
    public boolean initHuskyLens(HuskyLens huskyLens, PropColor findPropColor)
    {
        huskyLens_apollo = huskyLens;
        //Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.

        huskyLens_apollo = huskyLens;
        //Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

           */
        //rateLimit.expire();
        boolean intSucceeded;
        if (!huskyLens.knock()) {
            intSucceeded = false;
             Log.d(TAG_HUSKYLENS, "Problem communicating with " + huskyLens.getDeviceName() );
        } else {
            intSucceeded = true;
            Log.d(TAG_HUSKYLENS, "Press start to continue");
        }
        huskyLens_apollo.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        huskyLensState = HuskyLens_State.COLOR_RECOGNITION;
        propColor = findPropColor;
        Log.d(TAG_HUSKYLENS ,"find prop color " + propColor.toString());
        if (propColor == PropColor.RED)
        {
            propId = 2;
        }
        else {
            propId = 1;
        }
        return intSucceeded;
    }
    public PropPos detectPropPos()
    {
        PropPos propPos = null;
        blocks = huskyLens_apollo.blocks();
        if(blocks.length != 0)
        {
            Log.d(TAG_HUSKYLENS, " The number of blocks is " + blocks.length);
            numOfBlocks = blocks.length;
            for (int i = 0; (i < blocks.length) && (propPos == null); i++)
            {
                //play with top or left
                // id2 is the red one, id1 is the blue one.
                Log.d(TAG_HUSKYLENS, "the id of block" + i + "is" + blocks[i].id);
                Log.d(TAG_HUSKYLENS, "The position of block " + i + " is [x,y] (" + blocks[i].x + "," + blocks[i].y + ")");
                Log.d(TAG_HUSKYLENS,  ", top " + blocks[i].top + " , left " + blocks[i].left);
                Log.d(TAG_HUSKYLENS,  ", width " + blocks[i].width + " , height " + blocks[i].height);
                if ((propId == blocks[i].id))
                {
                    if ((blocks[i].height <= maxHeight) && (blocks[i].width <= maxWidth))
                    {
                        if ((blocks[i].x < middle) && (blocks[i].top < maxTop) && (blocks[i].top > minTop))
                        {
                            propPos = PropPos.UP;
                            Log.d(TAG_HUSKYLENS, "The Prop is on line Up");
                        }
                        else if ((blocks[i].x > middle)  && (blocks[i].top < maxTop) && (blocks[i].top > minTop))
                        {
                            propPos = PropPos.RIGHT;
                            Log.d(TAG_HUSKYLENS, "The Prop is on line Right");
                        }
                        else
                        {
                            propPos = null;
                            Log.d(TAG_HUSKYLENS, "No Prop was recognized");
                        }
                    }

                }
            }
        }
        return propPos;
    }
}
