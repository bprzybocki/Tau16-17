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


@Autonomous(name = "Tau: Button Blue", group = "Tau")
// @Autonomous(...) is the other common choice
//@Disabled
public class Auto_Button_Blue extends LinearOpMode {

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

        telemetry.addData("Status", "Initialized");

        //the below lines set up the configuration file
        colorA = robot.buttonSensorL;
        colorC = robot.buttonSensorR;

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();

        waitForStart(); //--------------------------------------------------------------------------



        if (LEDState) {
            colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
            //colorCreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        } else {
            colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
            //colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon
            while(runtime.milliseconds() < 10000.0) {
            colorAcache = colorAreader.read(0x04, 1);
            colorCcache = colorCreader.read(0x04, 1);


            //display values
            telemetry.addData("1 #A", colorAcache[0] & 0xFF);
            telemetry.addData("2 #C", colorCcache[0] & 0xFF);

            telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());
            telemetry.addData("4 A", colorCreader.getI2cAddress().get8Bit());
        }
        /*

        //SHOOT AND MOVE ---------------------------------------------------------------------------
        moveForwardAndShoot();

        //MOVE TO BUTTON 1 -------------------------------------------------------------------------
        moveToButton1();;
        /*
        if ((colorAcache[0] & 0xFF) == 2 || (colorAcache[0] & 0xFF) == 3 || (colorAcache[0] & 0xFF) == 4) {
            robot.buttonMotor.setPower(-0.1);
            sleepTau(300);
            robot.buttonMotor.setPower(0.1);
            sleepTau(300);
        }
        else
        {
            goStraightPower(-0.1);

        }*/

    }

    public void moveToButton1(){

    }

    public  void moveToButton2From1(){

    }


    public void moveForwardAndShoot()
    {
        robot.flywheelMotorL.setPower(robot.FLYWHEEL_PWR);
        robot.flywheelMotorR.setPower(robot.FLYWHEEL_PWR);
        goStraightPower(-0.5);
        sleepTau(550);
        goStraightPower(0);
        sleepTau(550);
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        sleepTau(550);
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
        sleepTau(1000);
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        sleepTau(550);
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
        robot.flywheelMotorL.setPower(0);
        robot.flywheelMotorR.setPower(0);

    }
    public void sleepTau(long millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public int getLeftPosition()
    {
        return (robot.leftFrontMotor.getCurrentPosition() + robot.leftBackMotor.getCurrentPosition()) / 2;
    }

    public int getRightPosition()
    {
        return (robot.rightFrontMotor.getCurrentPosition() + robot.rightBackMotor.getCurrentPosition()) / 2;
    }

    public void leftSetPower(double power)
    {
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
    }

    public void rightSetPower(double power)
    {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
    }
    public void goStraightPower(double power)
    {
        rightSetPower(power);
        leftSetPower(power);
    }
}

