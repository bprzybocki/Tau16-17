package org.firstinspires.ftc.teamcode;

/**
 * Created by BobChuckyJoe on 1/30/2017.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tau: Color/Distance", group = "Tau")
public class Color extends OpMode {
    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV  = 1440;
    static final double DRIVE_GEAR_REDUCTION  = 1.0 / 3;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED           = 0.6;
    static final double TURN_SPEED            = 0.5;
    static final double ERROR  = 1.5;
    static final int INTERIM_TIME = 100;
    static final double POWER = 0.3;

    static byte[] colorAcache;
    static byte[] colorCcache;

    static byte[] range1Cache;

    /*int zAccumulated;  //Total rotation left/right
    int heading;       //Heading left/right. Integer between 0 and 359
    int xVal, yVal, zVal;  //Momentary rate of rotation in three axis
    */


    static I2cDevice colorA;
    static I2cDevice colorC;
    static I2cDeviceSynch colorAreader;
    static I2cDeviceSynch colorCreader;
    int zAccumulated = 0;  //Total rotation left/right
    int heading = 0;       //Heading left/right. Integer between 0 and 359
    int xVal =0 , yVal=0, zVal=0;  //Momentary rate of rotation in three axis


    //TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    //boolean touchState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false

    @Override
    public void init() {
        robot.init(hardwareMap);


        /*colorA = robot.buttonSensorL;
        colorC = robot.buttonSensorR;

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();
        */
    }

    @Override
    public void loop() {
        telemetry.addData("HSV", getBeaconColor());
        telemetry.addData("DIST", getUltrasonicDistance());
        telemetry.update();
    }

    public int getBeaconColor(){
        colorAcache = robot.colorAreader.read(0x04, 1);
        return colorAcache[0] & 0xFF;
    }
    public int getUltrasonicDistance(){
        range1Cache = robot.RANGE1Reader.read(robot.RANGE1_REG_START, robot.RANGE1_READ_LENGTH);
        return range1Cache[0] & 0xFF;
    }
}