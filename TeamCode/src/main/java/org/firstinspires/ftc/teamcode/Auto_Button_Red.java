package org.firstinspires.ftc.teamcode;

/*
Modern Robotics Color Sensors Example with color number
Created 9/29/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.2
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "ca" (MRI Color Sensor with I2C address 0x3a (0x1d 7-bit)
I2CDevice "cc" (MRI Color Sensor with default I2C address 0x3c (0x1e 7-bit)

ModernRoboticsI2cColorSensor class is not being used because it can not access color number.
ColorSensor class is not being used because it can not access color number.

To change color sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


@Autonomous(name = "Tau: Button Red", group = "Tau")
// @Autonomous(...) is the other common choice
//@Disabled
public class Auto_Button_Red extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();

    byte[] colorAcache;
    byte[] colorCcache;

    I2cDevice colorA;
    I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorCreader;

    //TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    //boolean touchState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        colorA = hardwareMap.i2cDevice.get("sens_r");
        colorC = hardwareMap.i2cDevice.get("sens_l");

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();

        //touch = hardwareMap.touchSensor.get("t");

        waitForStart(); //--------------------------------------------------------------------------

        runtime.reset();

        if(LEDState){
            colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        }
        else{
            colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon

        colorAcache = colorAreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);

        //display values
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);

        telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());
        telemetry.addData("4 A", colorCreader.getI2cAddress().get8Bit());
    }
}