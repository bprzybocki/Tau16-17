package org.firstinspires.ftc.teamcode;

/**
 * Created by BobChuckyJoe on 1/30/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@Autonomous(name = "Tau: DERP", group = "Tau")

public class Auto_BLUE extends LinearOpMode{
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
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status","Calibrating");
        telemetry.update();
        robot.mrGyro.calibrate();
        while(robot.mrGyro.isCalibrating())
        {

        }
        telemetry.addData("Status","Done Calibrating");
        telemetry.update();

        /*colorA = robot.buttonSensorL;
        colorC = robot.buttonSensorR;

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();
        */
        robot.mrGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        robot.mrGyro.calibrate();
        waitForStart();

        FlywheelsOn();
        DriveStraightAbsolute(1.0,1.0);       //Speed of FULL POWER(1.0), One tile forward
        Shoot();
        Shoot();
        //0 is forward, negative degrees is Clockwise, positive is COUNTERCLOCKWISE
        TurnToAbsolute(0.8,-45);

        DriveStraightAbsolute(1.0,2.1);  //Drive halfway

        TurnToAbsolute(0.2,-45); //Readjust midway

        DriveStraightAbsolute(1.0,2.1);

        TurnToAbsolute(0.4,-90);

        DriveStraightUntilProximity(0.5,0.5);    //Drive UNTIL 0.5 tiles to wall

        TurnToAbsolute(0.4,0);

        DriveStraightUntilLine(1.0);

        DriveStraightBackwardsUntilLine(0.3);

        TurnToAbsolute(0.3,-90);

        DriveStraightUntilProximity(0.5,0.25);

        for(int i = 0; i < 2; i++)
        {
            DriveStraightUntilProximity(0.2,0.0);
            DriveStraightBackwards(0.1, 1/12);
            if(colorIs(BLUE)) {
                break;
            }
            sleepTau(5000);
        }
        DriveStraightBackwards(0.5, 0.5);

        TurnToAbsolute(0.5,0);

        DriveUntilLine(1.0);

        TurnToAbsolute(0.5,-90);

        for(int i = 0; i < 2; i++)
        {
            DriveStraightUntilProximity(0.2,0.0);
            DriveStraightBackwards(0.1, 1/12);
            if(colorIs(BLUE)) {
                break;
            }
            sleepTau(5000);
        }

        ParkMid();

        //Park middle later


/*


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
        goStraightPower(0.5,1000);
        sleepTau(1000);
        leftSetPower(0.1,500);


        //SHOOT AND MOVE ---------------------------------------------------------------------------
        moveForwardAndShoot();
        //MOVE TO BUTTON 1 -------------------------------------------------------------------------
        moveToButton1();
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

    //
    // DriveStraightAbsolute() - move the robot forward for the given number
    //                           of tiles at the given speed. The "straightness"
    //                           is based upon the given heading (negative CW
    //                           and positive CCW).
    //
    public void DriveStraightAbsolute(double speed, double tiles, int targetHeading)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int TICKS_PER_TILE = 600;           // number of encoder ticks per tile
        double ERROR_ADJUSTMENT = 0.02;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = -1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move

        int target_count = (int)(tiles*TICKS_PER_TILE);

        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        while (robot.leftMotor.getCurrentPosition()*LEFT_POLARITY < target_count) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(speed - error*ERROR_ADJUSTMENT);
            robot.rightMotor.setPower(speed + error*ERROR_ADJUSTMENT);
            telemetry.addData("Gyro Heading Raw", robot.mrGyro.getHeading());
            telemetry.addData("Gyro Error", error);
            telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
            telemetry.addData("Right Ticks", robot.rightMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Debug", "Exiting loop");
        telemetry.update();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    //
    // getHeading() - Read the gyro, and convert the heading to our heading where 0 is forward,
    //                negative is clockwise, and positive is counterclockwise.
    //
    public int getHeading() {
        int heading = robot.mrGyro.getHeading();
        if (heading < 180)
            return heading;
        else
            return -(360 - heading);
    }

    //This function turns a number of degrees compared to where the robot is. Positive numbers trn left.
    public void turn(int target) throws InterruptedException {
        turnAbsolute(target + robot.mrGyro.getIntegratedZValue());
    }


    public void turnAbsolute(int target) {
        zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
        double turnSpeed = 0.7;

        while (Math.abs(zAccumulated - target) > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (zAccumulated > target) {  //if gyro is positive, we will turn right
                robot.leftMotor.setPower(turnSpeed);
                robot.rightMotor.setPower(-turnSpeed);
            }

            if (zAccumulated < target) {  //if gyro is positive, we will turn left
                robot.leftMotor.setPower(-turnSpeed);
                robot.rightMotor.setPower(turnSpeed);
            }

            zAccumulated = robot.mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }

        robot.leftMotor.setPower(0);  //Stop the motors
        robot.rightMotor.setPower(0);

    }


    public void moveForwardAndShoot()
    {
        robot.flywheelMotorL.setPower(robot.FLYWHEEL_PWR);
        robot.flywheelMotorR.setPower(robot.FLYWHEEL_PWR);
        goStraightPower(-0.5,550);
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
        return robot.leftMotor.getCurrentPosition();
    }

    public int getRightPosition()
    {
        return robot.rightMotor.getCurrentPosition();
    }

    public void leftSetPower(double power, long mili)
    {
        robot.leftMotor.setPower(power);
        sleepTau(mili);
        robot.leftMotor.setPower(0);
    }

    public void rightSetPower(double power, long mili)
    {
        robot.rightMotor.setPower(power);
        sleepTau(mili);

        robot.rightMotor.setPower(0);
    }
    public void goStraightPower(double power, long mili)
    {
        rightSetPower(power,mili);
        leftSetPower(power,mili);

    }
    /*public void encoderDrive(double speed,
                             double leftInches, double rightInches,
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
*/
}