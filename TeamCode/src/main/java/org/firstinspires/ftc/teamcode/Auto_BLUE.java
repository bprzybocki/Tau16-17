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
/*
@Autonomous(name = "Tau: DERP", group = "Tau")
@Disabled
public class Auto_BLUE extends LinearOpMode{
    /* Declare OpMode members. */
/*    Hardware robot = new Hardware();
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

/*
    static I2cDevice colorA;
    static I2cDevice colorC;
    static I2cDeviceSynch colorAreader;
    static I2cDeviceSynch colorCreader;

    //TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    //boolean touchState = false;  //Tracks the last known state of the touch sensor
    boolean LEDState = false;     //Tracks the mode of the color sensor; Active = true, Passive = false

    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        int zAccumulated;  //Total rotation left/right
        int heading;       //Heading left/right. Integer between 0 and 359
        int xVal, yVal, zVal;  //Momentary rate of rotation in three axis

        GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
        ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        /*colorA = robot.buttonSensorL;
        colorC = robot.buttonSensorR;

        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        colorAreader.engage();
        colorCreader.engage();
        *//*
        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro)sensorGyro;
        //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();
        waitForStart();

        FlywheelsOn();
        DriveStraightAbsolute(1.0,1.0,0);       //Speed of FULL POWER(1.0), One tile forward
        Shoot();
        Shoot();
        //0 is forward, negative degrees is Clockwise, positive is COUNTERCLOCKWISE
        TurnToAbsolute(0.8,-45);

        DriveStraightAbsolute(1.0,2.1,-45);  //Drive halfway

        TurnToAbsolute(0.2,-45); //Readjust midway

        DriveStraight(1.0,2.1);

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











        telemetry.addData("Status","Calibrating");
        telemetry.update();
        while(mrGyro.isCalibrating())
        {

        }
        telemetry.addData("Status","Done Calibrating");
        telemetry.update();
        sleepTau(5000);
        mrGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        while(opModeIsActive()) {


            telemetry.addData("Status", "Running: " + runtime.toString());

            zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings
            heading = mrGyro.getHeading();
            /*heading = 360 - mrGyro.getHeading();  //Reverse direction of heading to match the integrated value
            if (heading == 360) {
                heading = 0;
            }
*//*
            xVal = mrGyro.rawX() / 128;  //Lowest 7 bits is noise
            yVal = mrGyro.rawY() / 128;
            zVal = mrGyro.rawZ() / 128;

            telemetry.addData("1. heading", String.format("%03d", heading));  //Display variables to Driver Station Screen
            telemetry.addData("2. accu", String.format("%03d", zAccumulated));
            telemetry.addData("3. X", String.format("%03d", xVal));
            telemetry.addData("4. Y", String.format("%03d", yVal));
            telemetry.addData("5. Z", String.format("%03d", zVal));
            telemetry.update();
            waitOneFullHardwareCycle();
        }
    }

    //
    // DriveStraightAbsolute() - move the robot forward for the given number
    //                           of tiles at the given speed. The "straightness"
    //                           is based upon the given heading (negative CW
    //                           and positive CCW).
    //
    public void DriveStraightAbsolute(float speed, float tiles, int targetHeading)
    {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int target_count = (int)(tiles*6000);
        int error;
        float ERROR_ADJUSTMENT = 0.2f;

        while (robot.leftFrontMotor.getCurrentPosition() < target_count) {
            error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftFrontMotor.setPower(speed - error*ERROR_ADJUSTMENT);
            robot.rightFrontMotor.setPower(speed + error*ERROR_ADJUSTMENT);
        }
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
    }

    //
    // getHeading() - Read the gyro, and convert the heading to our heading where 0 is forward,
    //                negative is clockwise, and positive is counterclockwise.
    //
    public int getHeading() {
        int heading = robot.mrGyro.getHeading();
        if (heading < 180)
            return -heading;
        else
            return 360 - heading;
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
        return robot.leftBackMotor.getCurrentPosition();
    }

    public int getRightPosition()
    {
        return robot.rightBackMotor.getCurrentPosition();
    }

    public void leftSetPower(double power, long mili)
    {
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        sleepTau(mili);
        robot.leftBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
    }

    public void rightSetPower(double power, long mili)
    {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        sleepTau(mili);

        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }
    public void goStraightPower(double power, long mili)
    {
        rightSetPower(power,mili);
        leftSetPower(power,mili);

    }

    public void encoderDrive(double speed,
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
}*/