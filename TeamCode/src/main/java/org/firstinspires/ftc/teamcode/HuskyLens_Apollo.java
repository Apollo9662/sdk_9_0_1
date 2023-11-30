package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class HuskyLens_Apollo
{
    final String TAG_HUSKYLENS = "HuskyLens_Apollo";
    //private final int READ_PERIOD = 1;
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
    public boolean initHuskyLens(HuskyLens huskyLens, PropColor findPropColor)
    {
        huskyLens_apollo = huskyLens;
        //Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
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
            propId = 1;
        }
        else {
            propId = 2;
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
            for (int i = 0; (i < blocks.length) && (propPos == null); i++)
            {
                // id1 is the red one, id2 is the blue one.
                Log.d(TAG_HUSKYLENS, "the id of block" + i + "is" + blocks[i].id);
                Log.d(TAG_HUSKYLENS, "The position of block " + i + " is [x,y] (" + blocks[i].x + "," + blocks[i].y + ")");
                if ((blocks[i].x > 88) && (blocks[i].x < 160) &&(blocks[i].y > 40) && (blocks[i].y< 60))
                {
                    propPos = PropPos.UP;
                    Log.d(TAG_HUSKYLENS, "The Prop is on line Top");
                }
                else if ((blocks[i].x > 217) && (blocks[i].x < 244) && (blocks[i].y < 192) && (blocks[i].y > 105))
                {
                    propPos = PropPos.RIGHT;
                    Log.d(TAG_HUSKYLENS, "The Prop is on line Right");
                }
                else if ((blocks[i].x > 217) && (blocks[i].x < 244) && (blocks[i].y < 192) && (blocks[i].y > 105))
                {
                    propPos = PropPos.LEFT;
                    Log.d(TAG_HUSKYLENS, "The Prop is on line Left");
                }
                else
                {
                   propPos = null;
                   Log.d(TAG_HUSKYLENS, "No Prop was recognized");
                }
            }
        }
        return propPos;
    }
}
