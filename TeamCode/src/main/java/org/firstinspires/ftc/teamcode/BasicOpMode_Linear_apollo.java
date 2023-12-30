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

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.util.List;


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
@TeleOp(name="TeleOp Linear apollo", group="TeleOp")
//@Disabled
public class BasicOpMode_Linear_apollo extends LinearOpMode {

    private GamepadEx driveOp;


    private ElapsedTime runtime = new ElapsedTime();
    public static double collectionSpeed = 0.8;
    boolean press = false;
    boolean pressCollection = false;
    boolean pressCollectionServo = false;
    boolean pressCollectionLift = false;
    boolean pressLift = false;
    boolean pressDrive = false;
    final int FIRST_LIFT = 855;
    final int SECOND_LIFT = 1710;
    final int THIRD_LIFT = 2565;
    final int FOURTH_LIFT = 3420;
    int liftMaxHight = 3420;
    final double POWER_LIFT = 1;
    double liftPower = 0;
    boolean inPosition = false;
    final String TAG_LIFT = "Lift";
    final String TAG_DRIVE = "Drive";
    final String TAG_COLLECTION_THREAD = "CollectionThread";
    final String TAG_LIFT_THREAD = "LiftThread";
    int currentPosition;
    final double LIFT_TIMEOUT_SEC = 5;
    final double LIFT_TIMEOUT_STAY_SEC = 2;
    boolean controlMod = false;
    boolean upSideDownMod = false;
    boolean doNotGoDownMod = false;
    CollectThread collectThread = new CollectThread();
    LiftThread liftTread = new LiftThread();
    enum LiftState {STOP,
                    MANUAL_CONTROL,
                    AUTO_CONTROL_WAIT_FOR_BUSY,
                    AUTO_CONTROL_IN_POSITION,
                    AUTO_CONTROL_ERROR,
                    RESETING_INCODER};

    int pos;
    boolean stayInPosIsActive = false;
    //ConceptTensorFlowObjectDetection_Apollo detection;
    RobotHardware_apollo robot = new RobotHardware_apollo();
    RobotHardware_apollo_FtcLib robot_Ftclib = new RobotHardware_apollo_FtcLib();


    @Override
    public void runOpMode() {
        driveOp = new GamepadEx(gamepad1);

        //armServoGardState = ArmServoGardState.CLOSE;
        robot.init(hardwareMap, false, false);
        robot_Ftclib.init(hardwareMap);
        robot.ServoInit();
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
          //      AxesOrder.ZYX, AngleUnit.DEGREES);
        //robot.initMecanumDriveBase();
        robot_Ftclib.SetAllMotorsZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.RunMode.RUN_USING_ENCODER);

        //detection = new ConceptTensorFlowObjectDetection_Apollo(this);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive"); //0
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive"); //1
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive"); //2
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");//3
        rightCollection = hardwareMap.get(DcMotor.class, "right_collection");//0
        leftCollection = hardwareMap.get(DcMotor.class, "left_collection");//1
        lift = hardwareMap.get(DcMotor.class, "lift");//2
        touchSensor1 = hardwareMap.get(TouchSensor.class, "sensor_touch1");
        touchSensor2 = hardwareMap.get(TouchSensor.class, "sensor_touch2");
        collectionServo = hardwareMap.get(Servo.class, "collection_servo");
        collectionGardServo = hardwareMap.get(Servo.class, "collection_gard_servo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightCollection.setDirection(DcMotorSimple.Direction.FORWARD);
        leftCollection.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCollection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionServo.setDirection(Servo.Direction.FORWARD);
        collectionGardServo.setDirection(Servo.Direction.FORWARD);


         */


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        collectThread.start();
        liftTread.start();




      // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {


            drive();
            /*
            if(robot.IsPressed(RobotHardware_apollo.DriveMotors.TOUCH_SENSOR1) == true)
            {
                telemetry.addData("Touch Sensor1", "Is Pressed");
            }
            else
            {
                telemetry.addData("Touch Sensor1", "Is Not Pressed");
            }
            if(robot.IsPressed(RobotHardware_apollo.DriveMotors.TOUCH_SENSOR2) == true)
            {                telemetry.addData("Touch Sensor2", "Is Pressed");
            }
            else
            {
                telemetry.addData
                ("Touch Sensor2", "Is Not Pressed");
            }

             */
            //boolean collectionSpeedMore = gamepad2.right_bumper;
            //boolean collectionSpeedLess = gamepad2.left_bumper;

          //  if (collectionSpeedMore == true)
            {
               // if (press == false)
                {
                    //press = true;
                   // if (collectionSpeed < 1)
                    {
                       // collectionSpeed += 0.1;
                    }
                }
            }

           // if (collectionSpeedLess== true)
            {
               // if (press == false)
                {
                   //press = true;
                   // if (collectionSpeed > 0.1)
                    {
                       // collectionSpeed -= 0.1;
                    }
                }
            }

            //if ((collectionSpeedLess == false) && (collectionSpeedMore == false))
            {
                //press = false;
            }






            // Show the elapsed game time and wheel power.

            if (true == inPosition )
            {
                telemetry.addLine("lift in position!!!");
            }
            //detection.detect();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MAXH is","(%.2f)" + liftMaxHight);
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            //telemetry.addData("side power", "(%.2f)",sidePower);
            telemetry.addData("lift Pos is ", "(%.2f)", robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
            telemetry.addData("collectionSpeed", "(%.2f)", collectionSpeed);
            telemetry.addData("servo pos is", "(%.2f)", robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO));
            telemetry.update();

        }
    }



    private void drive()
    {
        double forwardSpeed;
        double turnSpeed;
        double strafeSpeed;
        forwardSpeed   = -driveOp.getLeftY();  // Note: pushing stick forward gives negative value
        strafeSpeed =  driveOp.getLeftX();
        turnSpeed     =  -driveOp.getRightX();
        double heading = robot_Ftclib.getRobotYawPitchRollAngles();

        if ((driveOp.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) || driveOp.isDown((GamepadKeys.Button.RIGHT_STICK_BUTTON)))
        {
            controlMod = true;
        }
        else
        {
            controlMod = false;
        }
        if (controlMod == true)
        {
            robot_Ftclib.driveFieldCentric(strafeSpeed/2, forwardSpeed/2, turnSpeed/2, heading);
        }
        else
        {
            robot_Ftclib.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
        }



    }

    public class CollectThread extends Thread
    {
        public CollectThread()
        {
            this.setName("CollectThread");
            Log.d(TAG_COLLECTION_THREAD, "start collectThread");
        }


        @Override

        public void run()
        {
            try
            {

                while ((opModeIsActive()) && (!isInterrupted()))
                {
                    double collectionR = gamepad2.right_trigger;
                    double collectionF = gamepad2.left_trigger;
                    boolean collectPixel = gamepad2.left_bumper;
                    boolean dumpPixel =  gamepad2.right_bumper;
                    //boolean doNotGoDownSwitch = gamepad2.right_bumper;

                    if (collectPixel == true)
                    {
                        if (pressCollectionServo == false)
                        {
                            pressCollectionServo = true;
                            CollectPixel();
                            liftTread.goTo(0);
                        }
                    }

                    else if (dumpPixel == true)
                    {
                        if (pressCollectionServo == false)
                        {
                            pressCollectionServo = true;
                            DumpPixel();
                        /*
                        else if (armServoState == ArmServoState.OPEN)
                        {
                            armServoState = ArmServoState.CLOSE;
                            robot.SetPosition(RobotHardware_apollo.DriveMotors.COLLECTION_GARD_SERVO, 0.45);
                        }
                        */
                        }
                    }
                    else
                    {
                        pressCollectionServo = false;
                    }


                    if (collectionF > 0)
                    {
                            if (pressCollection == false)
                            {
                                pressCollection = true;
                                CollectPixel();
                                //liftTread.goTo(0);

                            }
                        collectionMotor(collectionSpeed);
                    }
                    else if (collectionR > 0)
                    {
                        collectionMotor(-collectionSpeed);
                    }
                    //else if ((detection.tfod.getRecognitions().size() < 0))
                    else
                    {
                        collectionMotor(0);
                    }
                }
            }  catch (Exception e)
            {
                Log.d(TAG_COLLECTION_THREAD, "catch exception: " + e.toString());
            }

        }



        public void collectionMotor (double speed)
        {
            robot.SetPower(RobotHardware_apollo.DriveMotors.COLLECTION, speed);
        }
        public void CollectPixel()
        {
            //74
            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO) != robot.ARM_SERVO_COLLECT_POS)
            {
                // log
                robot.armServoState = RobotHardware_apollo.ArmServoState.COLLECT;
                //armServoState = ArmServoState.COLLECT;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
            }
            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO) != robot.ARM_SERVO_GARD_OPEN_POS)
            {
                //log
                robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN;
                //armServoGardState = ArmServoGardState.OPEN;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, robot.ARM_SERVO_GARD_OPEN_POS);
            }
        }
        public void DumpPixel()
        {
            robot.armServoState = RobotHardware_apollo.ArmServoState.DUMP;
            //armServoState = ArmServoState.DUMP;
            robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_DUMP_POS);

            if(robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.CLOSE)
            {
                robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN_CLOSE;
                //armServoGardState = ArmServoGardState.OPEN_CLOSE;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, robot.ARM_SERVO_GARD_OPEN_CLOSE_POS);
            }
            else if(robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.OPEN_CLOSE)
            {
                robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.OPEN;
                //armServoGardState = ArmServoGardState.OPEN;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, robot.ARM_SERVO_GARD_OPEN_POS);
            }
            else if (robot.armServoGardState == RobotHardware_apollo.ArmServoGardState.OPEN)
            {
                robot.armServoGardState = RobotHardware_apollo.ArmServoGardState.CLOSE;
                //armServoGardState = ArmServoGardState.CLOSE;
                robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_GARD_SERVO, robot.ARM_SERVO_GARD_CLOSE_POS);
            }
        }
    }
    // ezra test check github
    private class LiftThread extends Thread
    {
        private LiftState liftState;
        private ElapsedTime liftTime = new ElapsedTime();
        private double timerTimeoutInSeconds;

        public LiftThread()
        {
            this.setName("LiftThread");
            Log.d(TAG_COLLECTION_THREAD, "start Lift thread");
        }
        public void run()
        {
            liftState = LiftState.STOP;
            liftTime.reset();
            Log.d(TAG_LIFT,"start");
            while ((opModeIsActive()) && (!isInterrupted()))
            {
                boolean liftUp = gamepad2.dpad_up;
                boolean liftDown = gamepad2.dpad_down;
                boolean liftPositionY = gamepad2.y;
                boolean liftPositionA = gamepad2.a;
                boolean liftPositionB = gamepad2.b;
                boolean liftPositionX = gamepad2.x;
                Log.d(TAG_LIFT,"liftPositionY " + liftPositionY);
                Log.d(TAG_LIFT,"liftPositionA " + liftPositionA);
                Log.d(TAG_LIFT,"liftPositionB " + liftPositionB);
                Log.d(TAG_LIFT,"liftPositionX " + liftPositionX);
                Log.d(TAG_LIFT,"pos is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));

                if (liftUp == false)
                {
                    resetIncoder();
                }
                if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY)
                {
                    if (isLiftBusy() == true)
                    {
                        /*
                        if (stayInPosIsActive == true)
                        {
                            Log.d(TAG_LIFT, "stay in position " + pos + " ; current position is " + lift.getCurrentPosition());
                            stayInPosIsActive = false;
                        }
                        else
                        {*/

                        Log.d(TAG_LIFT, "run to position " + pos + " ; current position is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                        //}

                    }
                    if (isLiftBusy() == false)
                    {
                        EndOfGetToPos();
                    }
                }
                if (liftUp == true)
                {
                    if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) < liftMaxHight)
                    {
                        liftState = LiftState.MANUAL_CONTROL;
                        Log.d(TAG_LIFT, "lift up pos " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                        robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_USING_ENCODER);
                        liftPower = POWER_LIFT;
                        inPosition = false;
                    }
                    else
                    {
                        if (inPosition == false)
                        {
                            liftPower = 0;
                            //stayInPos();
                        }
                    }

                }
                else if (liftDown == true)
                {
                    if (liftState != LiftState.RESETING_INCODER)
                    {
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, ARM_SERVO_COLLECT_POS);
                        collectThread.CollectPixel();
                        if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) >= 50)
                        {
                            liftState = LiftState.MANUAL_CONTROL;
                            Log.d(TAG_LIFT, "lift down pos " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_USING_ENCODER);
                            liftPower = -POWER_LIFT;
                            inPosition = false;
                        }
                        else
                        {
                            double distanceToZero = robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);
                            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT , (int) (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) - distanceToZero));
                            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_TO_POSITION);
                            liftPower = -POWER_LIFT;
                            robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT , liftPower);
                            //liftPower = 0;
                            if (inPosition == false)
                            {
                                //stayInPos();
                            }
                        }
                    }
                }
                else if (liftPositionY == true)
                {
                    if (pressLift == false)
                    {
                        collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        goTo(FIRST_LIFT);
                    }

                }
                else if (liftPositionB == true)
                {
                    if (pressLift == false)
                    {
                        collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        goTo(SECOND_LIFT);
                    }

                }
                else if (liftPositionA == true)
                {
                    if (pressLift == false)
                    {
                        collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        goTo(THIRD_LIFT);
                    }

                }
                else if (liftPositionX == true)
                {
                    if (pressLift == false)
                    {
                        collectThread.CollectPixel();
                        //robot.SetPosition(RobotHardware_apollo.DriveMotors.ARM_SERVO, robot.ARM_SERVO_COLLECT_POS);
                        pressLift = true;
                        goTo(FOURTH_LIFT);
                    }

                }
                else
                {
                    if ((inPosition == false) && (isLiftBusy() == false))
                    {
                        if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) >= 0)
                        {
                            stayInPos();
                        }

                    }
                    else if (isLiftBusy() == false)
                    {
                        liftState = LiftState.STOP;
                    }

                }
                if ((liftPositionA == false) && (liftPositionB == false) && (liftPositionX == false) && (liftPositionY == false) &&(gamepad2.right_bumper==false) && (gamepad2.left_bumper==false))
                {
                    pressLift = false;
                }

                if(liftPower != 0)
                {
                    Log.d(TAG_LIFT, "lift Power is " + liftPower);
                    Log.d(TAG_LIFT, "lift mode is  " + robot.lift.getMode());
                    Log.d(TAG_LIFT, "current position " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));

                }
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT , liftPower);


            }
            Log.d(TAG_LIFT,"finish");

        }
        public void resetIncoder()
        {

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
        public void goTo(int Pos)
        {
            if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY)
            {
                if (isLiftBusy() == true)
                {
                    liftrest();
                }
            }
            telemetry.addData("run to position ", "%d", Pos);
            Log.d(TAG_LIFT, "run to position " + Pos);
            pos = Pos;
            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_USING_ENCODER);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT , Pos);
            robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_TO_POSITION);
            liftPower = POWER_LIFT;
            robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT ,liftPower);
            inPosition = false;
            liftTime.reset();
            liftState = LiftState.AUTO_CONTROL_WAIT_FOR_BUSY;
            timerTimeoutInSeconds = LIFT_TIMEOUT_SEC;

        }
        private void stayInPos()
        {
            Log.d(TAG_LIFT,"stayInPos start");
            //liftState != LiftState.RESETING_INCODER ||
            if (robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT) > 20)
            {
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_USING_ENCODER);
                currentPosition = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);;
                currentPosition += 10;
                Log.d(TAG_LIFT, "stay in pos start, current Position is " + currentPosition);
                pos = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT);;
                //stayInPosIsActive = true;
                robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.LIFT , currentPosition);
                robot.SetMode(RobotHardware_apollo.DriveMotors.LIFT , DcMotor.RunMode.RUN_TO_POSITION);
                liftPower = POWER_LIFT;
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, liftPower);
                liftTime.reset();
                liftState = LiftState.AUTO_CONTROL_WAIT_FOR_BUSY;
                timerTimeoutInSeconds = LIFT_TIMEOUT_STAY_SEC;
            }
            else
            {
                robot.SetPower(RobotHardware_apollo.DriveMotors.LIFT, 0);
            }



        }
        private boolean isLiftBusy()
        {
            boolean isLiftBusy = false;
            robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT);
            if (liftState == LiftState.AUTO_CONTROL_WAIT_FOR_BUSY)
            {
                if ((robot.IsBusy(RobotHardware_apollo.DriveMotors.LIFT) == true) && (liftTime.seconds() < timerTimeoutInSeconds))
                {
                    isLiftBusy = true;
                }

            }
            return (isLiftBusy);
        }
        private void EndOfGetToPos()
        {
            if (liftTime.seconds() > timerTimeoutInSeconds)
            {

                Log.d(TAG_LIFT, "stopped due to time out. stopped at " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                liftState = LiftState.AUTO_CONTROL_ERROR;
                inPosition = true;

            }
            else
            {
                Log.d(TAG_LIFT, "Final Position is " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.LIFT));
                Log.d(TAG_LIFT, "timer in milliseconds is " + liftTime.milliseconds() + " timer in seconds is " + liftTime.seconds());
                inPosition = true;
                liftState = LiftState.AUTO_CONTROL_IN_POSITION;
            }
            robot.SetZeroPowerBehavior(RobotHardware_apollo.DriveMotors.LIFT, DcMotor.ZeroPowerBehavior.BRAKE);
        }
        private void liftrest()
        {
            stayInPos();
            Log.d(TAG_LIFT, "lift rest");


        }
    }



}
