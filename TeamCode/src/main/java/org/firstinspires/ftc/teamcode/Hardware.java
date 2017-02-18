package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 */
public class Hardware {

    public DcMotor leftMotor   = null;
    public DcMotor rightMotor = null;

    public DcMotor buttonMotor     = null;
    public DcMotor intakeMotor     = null;
    public DcMotor flywheelMotorR  = null;
    public DcMotor flywheelMotorL  = null;

    public Servo flyWheelPiston    = null;

    public GyroSensor sensorGyro   = null;
    public ModernRoboticsI2cGyro mrGyro = null;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    public static I2cDevice colorA;
    public static I2cDevice colorC;
    public static I2cDeviceSynch colorAreader;
    public static I2cDeviceSynch colorCreader;



    //public ModernRoboticsI2cGyro gyro = null;
    public I2cDevice beaconSensor = null;
    public I2cDevice groundSensor = null;
    //public static final double PISTON_DOWN = 0.33;
    //public static final double PISTON_UP = 0.83;
    public static final double PISTON_UP    = 0.2;
    public static final double PISTON_DOWN  = 1;
    public static final double FLYWHEEL_PWR = -0.3;
    public static final int FLYWHEEL_SPD    = 4000;
    public static final int COLOR_THRESHOLD = 96; //needs testing
    public static final int BLUE = 3;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public Hardware(){
    }

    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        period.reset();
        this.hwMap = hwMap;

        // Define and Initialize Motors

        leftMotor   = hwMap.dcMotor.get("left_front"); ///CORRECT!
        rightMotor = hwMap.dcMotor.get("right_front");
        buttonMotor     = hwMap.dcMotor.get("button");
        intakeMotor     = hwMap.dcMotor.get("intake");
        flywheelMotorL  = hwMap.dcMotor.get("flywheel_l");
        flywheelMotorR  = hwMap.dcMotor.get("flywheel_r");

        flywheelMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        beaconSensor = hwMap.i2cDevice.get("beacon_sens");
        groundSensor = hwMap.i2cDevice.get("ground_sens");

        sensorGyro = hwMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro)sensorGyro;

        RANGE1 = hwMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        colorA = beaconSensor;
        colorC = groundSensor;

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x3c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x5c), false);


        colorAreader.engage();
        colorCreader.engage();

        colorAreader.write8(3,1);
        colorCreader.write8(3,0);




        // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        // Set to FORWARD if using AndyMark motors
        buttonMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        buttonMotor.setPower(0);
        intakeMotor.setPower(0);
        flywheelMotorL.setPower(0);
        flywheelMotorR.setPower(0);
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //buttonMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //buttonSensorL.enableLed(true);
        //buttonSensorR.enableLed(true);
        // Just to check and make sure that it is not an encoder problem
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public double getTime()
    {
        return period.time();
    }
}