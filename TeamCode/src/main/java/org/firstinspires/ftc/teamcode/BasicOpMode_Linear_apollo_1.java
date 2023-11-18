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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Basic: Linear OpMode apollo 1", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear_apollo_1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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
    final int FIRST_LIFT = 1000;
    final double POWER_LIFT = 0.5;
    double stayPower = 0.01;
    boolean stay = false;
    int targetPos = 0;
    double liftPower = 0;

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double sidePower = gamepad1.left_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            double backLeftPower  = drive + turn - sidePower;
            double backRightPower = drive - turn + sidePower;
            double frontRightPower = drive - turn - sidePower;
            double frontLeftPower = drive + turn + sidePower;

            double maxFront = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            double maxBack = Math.max (Math.abs(backLeftPower), Math.abs(backRightPower));
            double max = Math.max(maxFront, maxBack);
            if (max > 1) {
                backLeftPower /= max;
                backRightPower /= max;
                frontLeftPower /= max;
                frontRightPower /= max;
            }

            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            frontRightDrive.setPower(frontRightPower);
            frontLeftDrive.setPower(frontLeftPower);

            boolean collectionSpeedMore = gamepad1.right_bumper;
            boolean collectionSpeedLess = gamepad1.left_bumper;

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


            double collectionF = gamepad1.right_trigger;
            double collectionR = gamepad1.left_trigger;

            if (collectionF > 0)
            {
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

            boolean liftUp = gamepad1.dpad_up;
            boolean liftDown = gamepad1.dpad_down;
            liftPower = 0;
            boolean liftPos0 = lift.getCurrentPosition() > 20;
            boolean liftPos900 = lift.getCurrentPosition() < 1020;

            if (liftUp == true && liftPos900 == true)
            {
                stay = false;
                liftPower = POWER_LIFT;
            }
            else if (liftDown == true && liftPos0 == true)
            {
                stay = false;
                liftPower = -POWER_LIFT;
            }





            boolean resetIncoder = gamepad1.b;

            if (resetIncoder == true)
            {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            boolean liftPositionY = gamepad1.y;

            if (liftPositionY == true)
            {
                telemetry.addData("run to position", "%d", FIRST_LIFT);
                lift.setTargetPosition(FIRST_LIFT);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower = POWER_LIFT;
            }
            if (lift.isBusy() == false)
            {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (lift.isBusy() == true) {
                liftPower = POWER_LIFT;
            }




            int getPos = lift.getCurrentPosition();
            telemetry.addData("get Pos", getPos);
            telemetry.addData("target Pos", targetPos);

            boolean switchStay = gamepad1.x;

            if (switchStay == true)
            {
                if (pressLift == false)
                {
                    targetPos = getPos;
                    pressLift = true;
                    stay = !stay;

                }

            }
            if (stay == true)
            {
                stayInPos(  getPos);

            }
            if (stay == false && lift.isBusy() == false)
            {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            lift.setPower(liftPower);

            if (switchStay == false)
            {
                pressLift = false;
            }



            // Show the elapsed game time and wheel power.
            int positionLift = lift.getCurrentPosition();

            telemetry.addData("lift Stay power","%f", stayPower);
            telemetry.addData("positionLift", "%f" , (float)positionLift);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("side power", "(%.2f)",sidePower);
            telemetry.addData("collectionSpeed", "(%.2f)", collectionSpeed);
            telemetry.update();

        }
    }

    private void collection (double speed)
    {
        leftCollection.setPower(speed);
        rightCollection.setPower(speed);
    }

public void stayInPos(double getPos)

    {
        lift.setTargetPosition(targetPos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (getPos > lift.getCurrentPosition())
        {
            liftPower = POWER_LIFT * (0.9 - Math.abs(getPos/targetPos));
        }

        if (getPos < lift.getCurrentPosition())
        {
            liftPower = -POWER_LIFT * (0.9 - Math.abs(getPos/targetPos));
        }
    }
}
