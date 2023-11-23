package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class HuskyLens_Apollo
{
    //private final int READ_PERIOD = 1;
    private boolean isPress = false;
    private enum HuskyLens_State {TAG_RECOGNITION,
        COLOR_RECOGNITION};
    public enum PropPos{UP,
                        RIGHT,
                        LEFT}
    HuskyLens.Block[] blocks;

    private HuskyLens_State huskyLensState;
    private HuskyLens huskyLens_apollo;
    public boolean initHuskyLens(HuskyLens huskyLens)
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
            //Log
            //elemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            intSucceeded = true;
            //Log
            //telemetry.addData(">>", "Press start to continue");
        }
        huskyLens_apollo.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        huskyLensState = HuskyLens_State.COLOR_RECOGNITION;
        return intSucceeded;
    }
    public PropPos detectPropPos()
    {
        PropPos propPos = null;
        blocks = huskyLens_apollo.blocks();
        if(blocks.length != 0)
        {
            //Log num of blocks
            for (int i = 0; (i < blocks.length) && (propPos == null); i++)
            {
                if ((blocks[i].x > 88) && (blocks[i].x < 160) &&(blocks[i].y > 40) && (blocks[i].y < 60))
                {
                    propPos = PropPos.UP;
                    //Log
                    //telemetry.addLine("The Prop is on line Top");
                }
                else if ((blocks[i].x > 217) && (blocks[i].x < 244) && (blocks[i].y < 192) && (blocks[i].y > 105))
                {
                    propPos = PropPos.RIGHT;
                    //Log
                    //telemetry.addLine("The Prop is on line right");
                }
                else if ((blocks[i].x > 217) && (blocks[i].x < 244) && (blocks[i].y < 192) && (blocks[i].y > 105))
                {
                    propPos = PropPos.LEFT;
                }
                else
                {
                   propPos = null;
                }
            }
        }
        return propPos;
    }
}
