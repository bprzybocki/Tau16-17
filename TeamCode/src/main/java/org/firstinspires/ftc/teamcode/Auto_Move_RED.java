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


@Autonomous(name = "Tau: Auto Shoot Move RED", group = "Tau")
// @Autonomous(...) is the other common choice
//@Disabled
public class Auto_Move_RED extends LinearOpMode {

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

    static byte[] colorAcache;
    static byte[] colorCcache;

    static I2cDevice colorA;
    static I2cDevice colorC;
    static I2cDeviceSynch colorAreader;
    static I2cDeviceSynch colorCreader;

    //TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    //boolean touchState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //the below lines set up the configuration file
        colorA = robot.buttonSensorL;
        colorC = robot.buttonSensorR;

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart(); //--------------------------------------------------------------------------



        if (LEDState) {
            colorAreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 0);    //Set the mode of the color sensor using LEDState
        } else {
            colorAreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
            colorCreader.write8(3, 1);    //Set the mode of the color sensor using LEDState
        }
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon
        //while(runtime.milliseconds() < 10000.0) {
        colorAcache = colorAreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);


        //display values
        //telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData("2 #C", colorCcache[0] & 0xFF);


        //telemetry.addData("3 A", colorAreader.getI2cAddress().get8Bit());
        telemetry.addData("4 C", colorCreader.getI2cAddress().get8Bit());
        telemetry.update();
        //encoderDrive(0.5,(int)(8 / 3 * Math.PI ),0,2);
        robot.flywheelMotorL.setPower(robot.FLYWHEEL_PWR + 0.0);
        robot.flywheelMotorR.setPower(robot.FLYWHEEL_PWR + 0.0);
        sleepTau(4000);

        //sleepTau(550);
        //goStraightPower(0);
        //sleepTau(550);
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        sleepTau(550);
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
        robot.intakeMotor.setPower(0.5);
        sleepTau(3750);
        robot.intakeMotor.setPower(0);
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        sleepTau(550);
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
        robot.flywheelMotorL.setPower(0);
        robot.flywheelMotorR.setPower(0);
        sleepTau(4000);
        runWithoutEncoders();
        leftSetPower(-0.2);
        rightSetPower(-0.3);
        sleepTau(1000);
        leftSetPower(0);
        rightSetPower(0);
        //encoderDrive(0.5,(int)(-16*ERROR),(int)(-16*ERROR),6);
        //rightSetPower(0.1);
        //sleepTau(1000);
        //rightSetPower(0);
        //}
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
    public void runWithoutEncoders()
    {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        return robot.leftBackMotor.getCurrentPosition();
    }

    public int getRightPosition()
    {
        return robot.rightBackMotor.getCurrentPosition();
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
    public void encoderDrive(double speed,
                             double
                                     leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {






            // Determine new target position, and pass to motor controller
            newLeftTarget = getLeftPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = getRightPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.leftBackMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition( newRightTarget);
            robot.rightBackMotor.setTargetPosition(  newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(-1 * Math.abs(speed));
            robot.rightBackMotor.setPower( -1 * Math.abs(speed));

            //rightSetPower(-1 * Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() &&
                            robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())
                    && Math.abs(robot.leftFrontMotor.getCurrentPosition()) < Math.abs(newLeftTarget) &&
                    Math.abs(robot.rightFrontMotor.getCurrentPosition()) < Math.abs(newRightTarget)) {

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);

            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //  sleep(250);   // optional pause after each move
            sleepTau(250);
        }
    }

}

