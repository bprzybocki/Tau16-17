package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Benjamin on 10/5/2016.
 */
public class Hardware {

    public DcMotor leftFrontMotor  = null;
    public DcMotor leftBackMotor   = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor  = null;
    public DcMotor buttonMotor     = null;
    public DcMotor intakeMotor     = null;
    public DcMotor flywheelMotorR  = null;
    public DcMotor flywheelMotorL  = null;

    public Servo flyWheelPiston    = null;

    //public ModernRoboticsI2cGyro gyro = null;
    public I2cDevice buttonSensorR = null;
    public I2cDevice buttonSensorL = null;
    //public static final double PISTON_DOWN = 0.33;
    //public static final double PISTON_UP = 0.83;
    public static final double PISTON_UP    = 0.2;
    public static final double PISTON_DOWN  = 1;
    public static final double FLYWHEEL_PWR = -0.6;
    public static final double FLYWHEEL_TELE = -0.5;
    public static final int FLYWHEEL_SPD    = 4000;
    public static final int COLOR_THRESHOLD = 96; //needs testing

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public Hardware(){
    }

    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        // Define and Initialize Motors
        leftFrontMotor  = hwMap.dcMotor.get("left_front");
        leftBackMotor   = hwMap.dcMotor.get("left_back");
        rightFrontMotor = hwMap.dcMotor.get("right_front");
        rightBackMotor  = hwMap.dcMotor.get("right_back");
        buttonMotor     = hwMap.dcMotor.get("button");
        intakeMotor     = hwMap.dcMotor.get("intake");
        flywheelMotorL  = hwMap.dcMotor.get("flywheel_l");
        flywheelMotorR  = hwMap.dcMotor.get("flywheel_r");

        flywheelMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        buttonSensorL = hwMap.i2cDevice.get("sens_l");
        buttonSensorR = hwMap.i2cDevice.get("sens_r");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        buttonMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        buttonMotor.setPower(0);
        intakeMotor.setPower(0);
        flywheelMotorL.setPower(0);
        flywheelMotorR.setPower(0);
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //buttonMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //buttonSensorL.enableLed(true);
        //buttonSensorR.enableLed(true);
        // Just to check and make sure that it is not an encoder problem
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Define and initialize ALL installed servos.
        flyWheelPiston = hwMap.servo.get("fly_piston");
        flyWheelPiston.setPosition(PISTON_DOWN);

        //flywheelMotorL.setMaxSpeed(FLYWHEEL_SPD);
        //flywheelMotorR.setMaxSpeed(FLYWHEEL_SPD);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
