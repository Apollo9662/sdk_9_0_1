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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    final String TAG_HARDWARE = "HardwareApollo";
    private HuskyLens huskyLens;
    private BNO055IMU imu = null;
    private TouchSensor touchSensor1 = null;
    private TouchSensor touchSensor2 = null;
    private Servo armServo = null;
    private Servo armGardServo = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor collection = null;
    public DcMotor lift = null; // private
    public DriveMotors driveMotors;
    public enum DriveMotors {BACK_LEFT_DRIVE,
            FRONT_LEFT_DRIVE,
            FRONT_RIGHT_DRIVE,
            BACK_RIGHT_DRIVE,
            ARM_SERVO,
            ARM_GARD_SERVO,
            TOUCH_SENSOR1,
            TOUCH_SENSOR2,
            LIFT,
            COLLECTION};
    final public double ARM_SERVO_COLLECT_POS = 0.71;
    final public double ARM_SERVO_DUMP_POS = 0.25;
    final public double ARM_SERVO_GARD_OPEN_POS = 0.15;
    final public double ARM_SERVO_GARD_CLOSE_POS = 0.45;
    final public double ARM_SERVO_GARD_OPEN_CLOSE_POS = 0.3;
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
    public void init(HardwareMap apolloHardwareMap)
    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        backLeftDrive = apolloHardwareMap.get(DcMotor.class, "back_left_drive"); //0
        frontLeftDrive = apolloHardwareMap.get(DcMotor.class, "front_left_drive"); //1
        backRightDrive = apolloHardwareMap.get(DcMotor.class, "back_right_drive"); //2
        frontRightDrive = apolloHardwareMap.get(DcMotor.class, "front_right_drive");//3
        collection = apolloHardwareMap.get(DcMotor.class, "collection");//0
        lift = apolloHardwareMap.get(DcMotor.class, "lift");//2
        touchSensor1 = apolloHardwareMap.get(TouchSensor.class, "sensor_touch1");
        touchSensor2 = apolloHardwareMap.get(TouchSensor.class, "sensor_touch2");
        armServo = apolloHardwareMap.get(Servo.class, "collection_servo");//0
        armGardServo = apolloHardwareMap.get(Servo.class, "collection_gard_servo");
        huskyLens = apolloHardwareMap.get(HuskyLens.class, "huskylens");

        imu = apolloHardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        if (imu.initialize(parameters) == false)
        {
            imu = apolloHardwareMap.get(BNO055IMU.class, "imu2");
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            if (imu.initialize(parameters) == false)
            {
                Log.d(TAG_HARDWARE, "initialization of imu2 failed");
            }
            else
            {
                Log.d(TAG_HARDWARE, "initialization of imu2 succeeded");
            }
        }
        else
        {
            Log.d(TAG_HARDWARE, "initialization of imu succeeded");
        }

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        collection.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
        armGardServo.setDirection(Servo.Direction.FORWARD);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collection.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void ServoInit()
    {
        armServo.setPosition(ARM_SERVO_COLLECT_POS);
        armGardServo.setPosition(ARM_SERVO_GARD_OPEN_POS);
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
            case COLLECTION:
            {
                return collection.getPower();
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
            case LIFT:
            {
                return (lift.getCurrentPosition());
            }
            case ARM_SERVO:
            {
                return (armServo.getPosition());
            }
            case ARM_GARD_SERVO:
            {
                return (armGardServo.getPosition());
            }
            default:
                return (1);
        }
    }
        public void SetTargetPosition(DriveMotors motor, int Position)
        {
            switch (motor) {

                case BACK_LEFT_DRIVE: {
                    backLeftDrive.setTargetPosition(Position);
                }
                break;
                case BACK_RIGHT_DRIVE: {
                    backRightDrive.setTargetPosition(Position);
                }
                break;
                case FRONT_LEFT_DRIVE: {
                    frontLeftDrive.setTargetPosition(Position);
                }
                break;
                case FRONT_RIGHT_DRIVE: {
                    frontRightDrive.setTargetPosition(Position);
                }
                break;
                case LIFT: {
                    lift.setTargetPosition(Position);
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
            case LIFT: {
                lift.setMode(myMode);
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

            case ARM_SERVO:
                armServo.setPosition(Position);
            break;
            case ARM_GARD_SERVO:
                armGardServo.setPosition(Position);
            break;
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


