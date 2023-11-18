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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="Basic: Linear OpMode apollo 2", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear_apollo2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime liftTime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor leftCollection = null;
    private DcMotor rightCollection = null;
    private DcMotor lift = null;
    private  double collectionSpeed = 0.8;
    boolean press = false;
    boolean pressLift = false;
    final int FIRST_LIFT = 220;
    final int SECOND_LIFT = 800;
    final int THIRD_LIFT = 1320;
    final int FOURTH_LIFT = 1780;
    final double POWER_LIFT = 0.5;
    double stayPower = 0.01;
    boolean stay = false;
    int targetPos = 0;
    double liftPower = 0;
    boolean inPosition = false;
    double liftFloor = 1;
    String TAG_LIFT = "lift";
    String TAG_DRIVE = "drive";
    int currentPosition;
    final double LIFT_TIMEOUT_SEC = 5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rightCollection = hardwareMap.get(DcMotor.class, "right_collection");
        leftCollection = hardwareMap.get(DcMotor.class, "left_collection");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightCollection.setDirection(DcMotorSimple.Direction.FORWARD);
        leftCollection.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        liftTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
           drive();

            boolean collectionSpeedMore = gamepad2.right_bumper;
            boolean collectionSpeedLess = gamepad2.left_bumper;

            if (collectionSpeedMore == true)
            {
                if (press == false)
                {
                    press = true;
                    if (collectionSpeed < 1)
                    {
                        collectionSpeed += 0.1;
                    }
                }
            }

            if (collectionSpeedLess== true)
            {
                if (press == false)
                {
                    press = true;
                    if (collectionSpeed > 0.1)
                    {
                        collectionSpeed -= 0.1;
                    }
                }
            }

            if ((collectionSpeedLess == false) && (collectionSpeedMore == false))
            {
                press = false;
            }


            double collectionF = gamepad2.right_trigger;
            double collectionR = gamepad2.left_trigger;

            if (collectionF > 0)
            {
                if (lift.getCurrentPosition() > 20)
                {
                    goTo(0);
                }
                collection(collectionF);
            }
            else if (collectionR > 0)
            {
                collection(-collectionR);
            }
            else
            {
               collection(0);
            }

            boolean liftUp = gamepad2.dpad_up;
            boolean liftDown = gamepad2.dpad_down;
            //liftPower = 0;
            boolean liftPositionY = gamepad2.y;
            boolean liftPositionA = gamepad2.a;
            boolean liftPositionB = gamepad2.b;
            boolean liftPositionX = gamepad2.x;
            //boolean resetIncoder = gamepad2.b;
            if (liftUp == true)
            {
                if (lift.getCurrentPosition() < 2400)
                {
                    //stay = false;
                    Log.d(TAG_LIFT, "lift up pos " + lift.getCurrentPosition());
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftPower = POWER_LIFT;
                    inPosition = false;
                }
                else
                {
                    if (inPosition == false)
                    {
                        stayInPos();
                    }
                }

            }
            else if (liftDown == true)
            {
                if (lift.getCurrentPosition() > 100)
                {
                    Log.d(TAG_LIFT, "lift down pos " + lift.getCurrentPosition());
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftPower = -POWER_LIFT;
                    inPosition = false;
                }
                else
                {
                    if (inPosition == false)
                    {
                        stayInPos();
                    }
                }

            }
            else if (liftPositionY == true)
            {
                telemetry.addData("run to position ", "%d", 0);
                Log.d(TAG_LIFT, "run to position 0");
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower = POWER_LIFT;
                lift.setPower(liftPower);
                inPosition = false;
                liftTime.reset();
                while ((lift.isBusy() == true) && (opModeIsActive()) && (liftTime.seconds() <= LIFT_TIMEOUT_SEC))
                {
                    telemetry.addData("lift Pos ", lift.getCurrentPosition());
                    Log.d(TAG_LIFT, "lift try to get to pos 0 ; current pos is  " + lift.getCurrentPosition());
                }
                Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
                if (liftTime.seconds() > LIFT_TIMEOUT_SEC)
                {
                    Log.d(TAG_LIFT, "stop due to time out");

                }
                inPosition = true;
                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //liftPower = 0;
            }
            else if (liftPositionA == true)
            {
                if (liftFloor == 1)
                {
                    telemetry.addData("run to position", "%d", FIRST_LIFT);
                    Log.d(TAG_LIFT, "run to position " + FIRST_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setTargetPosition(FIRST_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftPower = POWER_LIFT;
                    lift.setPower(liftPower);
                    inPosition = false;
                    liftTime.reset();
                    while (lift.isBusy() == true && opModeIsActive() && liftTime.seconds() <= LIFT_TIMEOUT_SEC)
                    {
                        telemetry.addData("lift Pos ", lift.getCurrentPosition());
                        Log.d(TAG_LIFT, "lift try to get to " + FIRST_LIFT + " Current Position is "+ lift.getCurrentPosition());
                    }
                    Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
                    if (liftTime.seconds() > LIFT_TIMEOUT_SEC)
                    {
                        Log.d(TAG_LIFT, "stop due to time out");

                    }
                    inPosition = true;
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else if (liftFloor == 2)
                {
                    telemetry.addData("run to position", "%d", SECOND_LIFT);
                    Log.d(TAG_LIFT, "run to position " + SECOND_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setTargetPosition(SECOND_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftPower = POWER_LIFT;
                    lift.setPower(liftPower);
                    inPosition = false;
                    liftTime.reset();
                    while (lift.isBusy() == true && opModeIsActive() && liftTime.seconds() <= LIFT_TIMEOUT_SEC)
                    {
                        telemetry.addData("lift Pos ", lift.getCurrentPosition());
                        Log.d(TAG_LIFT, "lift try to get to " + SECOND_LIFT + " Current Position is "+ lift.getCurrentPosition());
                    }
                    Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
                    if (liftTime.seconds() > LIFT_TIMEOUT_SEC)
                    {
                        Log.d(TAG_LIFT, "stop due to time out");

                    }
                    inPosition = true;
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else if (liftFloor == 3)
                {
                    telemetry.addData("run to position", "%d", THIRD_LIFT);
                    Log.d(TAG_LIFT, "run to position " + THIRD_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setTargetPosition(THIRD_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftPower = POWER_LIFT;
                    lift.setPower(liftPower);
                    inPosition = false;
                    liftTime.reset();
                    while (lift.isBusy() == true && opModeIsActive() && liftTime.seconds() <= LIFT_TIMEOUT_SEC)
                    {
                        telemetry.addData("lift Pos ", lift.getCurrentPosition());
                        Log.d(TAG_LIFT, "lift try to get to " + THIRD_LIFT + " Current Position is "+ lift.getCurrentPosition());
                    }
                    Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
                    if (liftTime.seconds() > LIFT_TIMEOUT_SEC)
                    {
                        Log.d(TAG_LIFT, "stop due to time out");

                    }
                    inPosition = true;
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else if (liftFloor == 4)
                {
                    telemetry.addData("run to position", "%d", FOURTH_LIFT);
                    Log.d(TAG_LIFT, "run to position " + FOURTH_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setTargetPosition(FOURTH_LIFT);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftPower = POWER_LIFT;
                    lift.setPower(liftPower);
                    inPosition = false;
                    liftTime.reset();
                    while (lift.isBusy() == true && opModeIsActive() && liftTime.seconds() <= LIFT_TIMEOUT_SEC)
                    {
                        telemetry.addData("lift Pos ", lift.getCurrentPosition());
                        Log.d(TAG_LIFT, "lift try to get to " + FOURTH_LIFT + " Current Position is "+ lift.getCurrentPosition());
                    }
                    Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
                    if (liftTime.seconds() > LIFT_TIMEOUT_SEC)
                    {
                        Log.d(TAG_LIFT, "stop due to time out");

                    }
                    inPosition = true;
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            else if (liftPositionX == true)
            {
                if (pressLift == false && liftFloor < 4)
                {
                    pressLift = true;
                    liftFloor += 1;
                }
            }
            else if (liftPositionB == true)
            {
                if (pressLift == false && liftFloor > 1)
                {
                    pressLift = true;
                    liftFloor -= 1;
                }
            }



            else
            {
                if (inPosition == false && liftUp == false && liftDown == false)
                {
                   stayInPos();
                }

            }
            if (liftPositionB == false && liftPositionX == false)
            {
                pressLift = false;
            }


            lift.setPower(liftPower);



            //if (lift.isBusy() == false)
            {
                //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //if (lift.isBusy() == true) {
                //liftPower = POWER_LIFT;
            //}




            int getPos = lift.getCurrentPosition();
            //telemetry.addData("get Pos", getPos);
            //telemetry.addData("target Pos", targetPos);

            boolean switchStay = gamepad2.x;

            //if (switchStay == true)
            //{
            //    if (pressLift == false)
           //     {
            //        targetPos = getPos;
            //        stay = !stay;
//
            //    }
//
            //}
            //if (stay == true)
            //{
            //        lift.setTargetPosition(targetPos);
            //        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
             //       if (getPos > lift.getCurrentPosition())
             //       {
             //           liftPower = POWER_LIFT * (0.9 - Math.abs(getPos/targetPos));
            //        }
//
            //        if (getPos < lift.getCurrentPosition())
            //        {
            //            liftPower = -POWER_LIFT * (0.9 - Math.abs(getPos/targetPos));
            //        }
           // }
            //if (stay == false && lift.isBusy() == false)
            //{
            //    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //}


            //if (switchStay == false)
            //{
            //    pressLift = false;
            //}



            // Show the elapsed game time and wheel power.
            int positionLift = lift.getCurrentPosition();

            telemetry.addData("lift floor", liftFloor);
            //telemetry.addData("positionLift", "%f" , (float)positionLift);
            if (true == inPosition )
            {
                telemetry.addLine("lift in position!!!");
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            //telemetry.addData("side power", "(%.2f)",sidePower);
            telemetry.addData("collectionSpeed", "(%.2f)", collectionSpeed);
            telemetry.update();

        }
    }

    private void collection (double speed)
    {
        leftCollection.setPower(speed);
        rightCollection.setPower(speed);
    }
    private void stayInPos()
    {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentPosition = lift.getCurrentPosition();
        currentPosition += 10;
        Log.d(TAG_LIFT, "stay in pos start, current Position is " + currentPosition);
        if (currentPosition > 0)
        {
            lift.setTargetPosition(currentPosition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftPower = POWER_LIFT;
            lift.setPower(liftPower);
            liftTime.reset();
            while (lift.isBusy() == true && opModeIsActive() && liftTime.seconds() <= 5)
            {
                telemetry.addData("lift Pos ", lift.getCurrentPosition());
                Log.d(TAG_LIFT, "say in pos " + lift.getTargetPosition() + " current pos is " + lift.getCurrentPosition());
            }
        }
        Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
        inPosition = true;
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void goTo(int Pos)
    {
        telemetry.addData("run to position ", "%d", Pos);
        Log.d(TAG_LIFT, "run to position " + Pos);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setTargetPosition(Pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftPower = POWER_LIFT;
        lift.setPower(liftPower);
        inPosition = false;
        liftTime.reset();
        while ((lift.isBusy() == true) && (opModeIsActive()) && (liftTime.seconds() <= LIFT_TIMEOUT_SEC))
        {
            drive();
            telemetry.addData("lift Pos ", lift.getCurrentPosition());
            Log.d(TAG_LIFT, "lift try to get to pos " + Pos + " current pos is " + lift.getCurrentPosition());
        }
        if (liftTime.seconds() > LIFT_TIMEOUT_SEC)
        {
            Log.d(TAG_LIFT, "stopped due to time out. stopped at " + lift.getCurrentPosition());

        }
        else
        {
            Log.d(TAG_LIFT, "Current Position is " + lift.getCurrentPosition());
        }
        inPosition = true;
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void drive()
    {
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double sidePower = gamepad1.left_stick_x;


        double backLeftPower  = drive + turn - sidePower;
        double backRightPower = drive - turn + sidePower;
        double frontRightPower = drive - turn - sidePower;
        double frontLeftPower = drive + turn + sidePower;

        double maxFront = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        double maxBack = Math.max (Math.abs(backLeftPower), Math.abs(backRightPower));
        double max = Math.max(maxFront, maxBack);
        if (max > 1)
        {
            backLeftPower /= max;
            backRightPower /= max;
            frontLeftPower /= max;
            frontRightPower /= max;
        }

        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
        frontRightDrive.setPower(frontRightPower);
        frontLeftDrive.setPower(frontLeftPower);

    }

}
