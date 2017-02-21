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

@Autonomous(name = "Full Blue", group = "Tau")
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
    static final int INTERIM_TIME = 100;
    static final double FAST_POWER = 0.5;
    static final double POWER = 0.40;
    static final double LINE_POWER = 0.2; // Slower because we don't want to miss line

    static byte[] colorAcache;
    static byte[] colorBcache;
    static byte[] colorCcache;

    static byte[] range1Cache;

    /*int zAccumulated;  //Total rotation left/right
    int heading;       //Heading left/right. Integer between 0 and 359
    int xVal, yVal, zVal;  //Momentary rate of rotation in three axis
    */


    static I2cDevice colorA;
    static I2cDevice colorB;
    static I2cDevice colorC;
    static I2cDeviceSynch colorAreader;
    static I2cDeviceSynch colorBreader;
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
        while(robot.mrGyro.isCalibrating() && opModeIsActive())
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
        waitForStart();

        //FlywheelsOn();

        //CODE TO KEEP!!!
        DriveStraightAbsolute(FAST_POWER,0.4,0);       //Speed of FULL POWER(1.0), One tile forward
        //Shoot();
        //Shoot();
        //FlywheelsOff();
        //0 is forward, negative degrees is Clockwise, positive is COUNTERCLOCKWISE
        sleepTau(INTERIM_TIME);

        TurnToAbsolute(-30);

        sleepTau(INTERIM_TIME);

        DriveStraightAbsolute(FAST_POWER,2.8,-30);  //Drive halfway

        sleepTau(INTERIM_TIME);

        TurnToAbsolute(-90);

        //PART TWO OF CODE!!!

        //THROWAWAY CODE
        //TurnToAbsolute(-90);

        sleepTau(INTERIM_TIME);

        telemetry.addData("Debug", getHeading());

        DriveStraightUntilProximity(POWER,-90,35,10000);    //Drive UNTIL 0.5 tiles to wall

        sleepTau(INTERIM_TIME);

        telemetry.addData("Debug", getHeading());

        TurnToAbsolute(0);

        sleepTau(INTERIM_TIME);

        DriveStraightUntilLine(LINE_POWER, 0);

        sleepTau(INTERIM_TIME);

        DriveStraightBackwards(POWER, 0.07, 0);

        sleepTau(INTERIM_TIME);

        TurnToAbsolute(-90);

        sleepTau(INTERIM_TIME);

        FlywheelsOn();

        for(int i = 0; i < 2; i++)
        {
            DriveStraightUntilProximity(POWER,-90,0,1000);
            telemetry.addData("Debug", "Driving Backwards");
            sleepTau(INTERIM_TIME);
            DriveStraightBackwards(POWER,0.17,-90);
            telemetry.addData("Beacon R", getBeaconColor_R());
            telemetry.addData("Debug", "Reading Beacon");
            if(colorIs(robot.COLOR_IS_BLUE) || colorIs(robot.COLOR_IS_BOTH) || colorIs(robot.COLOR_IS_NONE)) {
                break;
            }
            if (i == 0)
                sleepTau(5000);
        }

        telemetry.addData("Debug", "Turning");
        sleepTau(INTERIM_TIME);

        TurnToAbsolute(0);

        sleepTau(INTERIM_TIME);

        DriveBackwardsUntilLine(LINE_POWER+0.1,0);

        sleepTau(INTERIM_TIME);

        DriveStraightAbsolute(POWER, 0.12, 0);

        sleepTau(INTERIM_TIME);

        TurnToAbsolute(-90);

        sleepTau(INTERIM_TIME);

        //THROWAWAY CODE
        //TurnToAbsolute(-90);
        //FlywheelsOn();
        //sleepTau(5000);

        for (int i = 0; i < 2; i++)
        {
            DriveStraightUntilProximity(POWER,-90,0,1000);
            sleepTau(INTERIM_TIME);
            DriveStraightBackwards(POWER,0.17,-90);
            telemetry.addData("Beacon R", getBeaconColor_R());
            if (i == 0) {
                DoubleShoot(); // takes 4 seconds
            }
            if (colorIs(robot.COLOR_IS_BLUE) || colorIs(robot.COLOR_IS_BOTH) || colorIs(robot.COLOR_IS_NONE)) {
                sleepTau(INTERIM_TIME);
                break;
            }
            if (i == 0)
                sleepTau(1000); // sleep 1 more second to let beacon reset
        }


        DriveStraightBackwards(FAST_POWER,2,-90);

        PistonStow();

        //sleepTau(INTERIM_TIME);

        //ParkMid();
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
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int TICKS_PER_TILE = 1900;           // number of encoder ticks per tile
        double ERROR_ADJUSTMENT = 0.035;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = -1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move

        int target_count = (int)(tiles*TICKS_PER_TILE);

        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        while (robot.leftMotor.getCurrentPosition()*LEFT_POLARITY < target_count && opModeIsActive()) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(Math.max(speed - error*ERROR_ADJUSTMENT, 0)); //don't go below 0
            robot.rightMotor.setPower(Math.min(speed + error*ERROR_ADJUSTMENT, 1)); //don't go above 1
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
        TurnToAbsolute(target + robot.mrGyro.getIntegratedZValue());
    }

    public void TurnToAbsolute(int target) {
        int heading = getHeading();  //Set variables to gyro readings
        //double SPD_ADJUSTMENT = 0.006;
        int diff = Math.abs(heading - target);
        int THRESHOLD = 14;

        while (diff > THRESHOLD && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (heading > target) {  //if gyro is positive, we will turn right
                robot.leftMotor.setPower(0.4);
                robot.rightMotor.setPower(-0.4); //clockwise (slower, needs higher power)
            }

            if (heading < target) {  //if gyro is positive, we will turn left
                robot.leftMotor.setPower(-0.4); //counterclockwise
                robot.rightMotor.setPower(0.4);
            }

            heading = getHeading();  //Set variables to gyro readings
            telemetry.addData("heading", String.format("%03d", heading));
            telemetry.update();
            diff = Math.abs(heading - target);
        }

        robot.leftMotor.setPower(0);  //Stop the motors
        robot.rightMotor.setPower(0);
    }
    public void sleepTau(long millis)
    {
        double startTime = robot.getTime();

        double targetTime = startTime + ((double) millis) / 1000;

        while (robot.getTime() < targetTime && opModeIsActive())
        {
            try {
                idle();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
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
    public void FlywheelsOn()
    {
        robot.flywheelMotorL.setPower(robot.FLYWHEEL_AUTO);
        robot.flywheelMotorR.setPower(robot.FLYWHEEL_AUTO);
    }
    public void FlywheelsOff()
    {
        robot.flywheelMotorL.setPower(0);
        robot.flywheelMotorR.setPower(0);
    }

    public void Shoot()
    {
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        sleepTau(1000);
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
    }

    public void DoubleShoot()
    {
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        sleepTau(1000);
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
        sleepTau(3000);
        robot.flyWheelPiston.setPosition(robot.PISTON_UP);
    }

    public void PistonStow()
    {
        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
    }

    public int getBeaconColor_R(){
        //return 0;
        colorAcache = robot.colorAreader.read(0x04, 1);
        return colorAcache[0] & 0xFF;
    }
    public int getBeaconColor_L(){
        //return 0;
        colorBcache = robot.colorBreader.read(0x04, 1);
        return colorBcache[0] & 0xFF;
    }

    public int getGroundColor(){
        colorCcache = robot.colorCreader.read(0x04,1);
        return colorCcache[0] & 0xFF;
    }

    public void DriveStraightUntilLine(double speed, int targetHeading) {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean foundLine = false;
        double ERROR_ADJUSTMENT = 0.035;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = -1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move

        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        while (!foundLine && opModeIsActive()) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(Math.max(speed - error*ERROR_ADJUSTMENT, 0)); //don't go below 0
            robot.rightMotor.setPower(Math.min(speed + error*ERROR_ADJUSTMENT, 1)); //don't go above 1

            if (getGroundColor() > 13)
                foundLine = true;
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
    public void DriveBackwardsUntilLine(double speed, int targetHeading) {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean foundLine = false;
        double ERROR_ADJUSTMENT = 0.035;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = -1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move

        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        while (!foundLine && opModeIsActive()) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(-1*Math.min(speed + error*ERROR_ADJUSTMENT, 1)); //don't go below 0
            robot.rightMotor.setPower(-1*Math.max(speed - error*ERROR_ADJUSTMENT, 0)); //don't go above 1

            if (getGroundColor() > 13)
                foundLine = true;
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

    public void DriveStraightBackwards(double speed, double tiles, int targetHeading)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int TICKS_PER_TILE = 1900;           // number of encoder ticks per tile
        double ERROR_ADJUSTMENT = 0.035;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = 1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move

        int target_count = (int)(tiles*TICKS_PER_TILE);

        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        while (robot.leftMotor.getCurrentPosition()*LEFT_POLARITY < target_count && opModeIsActive()) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(-1*Math.min(speed + error*ERROR_ADJUSTMENT, 1)); //don't go below 0
            robot.rightMotor.setPower(-1*Math.max(speed - error*ERROR_ADJUSTMENT, 0)); //don't go above 1
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
    public void DriveStraightUntilProximity(double speed, int targetHeading, int distance, long timeoutMiliseconds){

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int TICKS_PER_TILE = 1900;           // number of encoder ticks per tile
        double ERROR_ADJUSTMENT = 0.035;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = -1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move


        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();
        double startTime = runtime.milliseconds();

        while (getUltrasonicDistance() > distance && runtime.milliseconds() - startTime < timeoutMiliseconds && opModeIsActive()) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(Math.max(speed - error*ERROR_ADJUSTMENT, 0)); //don't go below 0
            robot.rightMotor.setPower(Math.min(speed + error*ERROR_ADJUSTMENT, 1)); //don't go above 1
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
    public void DriveBackwardsUntilProximity(double speed, int targetHeading, int distance, long timeoutMiliseconds){

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int TICKS_PER_TILE = 1900;           // number of encoder ticks per tile
        double ERROR_ADJUSTMENT = 0.035;     // motor power adjustment per degree off of straight
        int LEFT_POLARITY = -1;     // encoder for the REV motors goes negative when moving forward
        //    may need to set to 1 for a different motor/encoder to keep
        //    the encoder values always positive for a forward move


        telemetry.addData("Debug", "Entering loop");
        telemetry.update();

        telemetry.addData("Left Ticks", robot.leftMotor.getCurrentPosition());
        telemetry.update();
        double startTime = runtime.milliseconds();

        while (getUltrasonicDistance() < distance && runtime.milliseconds() - startTime < timeoutMiliseconds && opModeIsActive()) {
            int error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftMotor.setPower(-Math.min(speed + error*ERROR_ADJUSTMENT, 1)); //don't go below 0
            robot.rightMotor.setPower(-Math.max(speed - error*ERROR_ADJUSTMENT, 0)); //don't go above 1
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
    public int getUltrasonicDistance(){
        range1Cache = robot.RANGE1Reader.read(robot.RANGE1_REG_START, robot.RANGE1_READ_LENGTH);
        return range1Cache[0] & 0xFF;
    }
    public int getODSDistance(){
        range1Cache = robot.RANGE1Reader.read(robot.RANGE1_REG_START, robot.RANGE1_READ_LENGTH);
        return range1Cache[1] & 0xFF;
    }
    public boolean colorIs(byte sensor, byte color)
    {
        int sensor_reading = (sensor == robot.LEFT_COLOR) ? getBeaconColor_L() : getBeaconColor_R();

        if (color == robot.COLOR_IS_BLUE) {
            return Math.abs(sensor_reading - robot.BLUE) <= 1;
        } else if (color == robot.COLOR_IS_RED) {
            return Math.abs(sensor_reading - robot.RED) <= 1;
        } else if (color == robot.COLOR_IS_NONE) {
            return !colorIs(sensor, robot.COLOR_IS_BLUE) && !colorIs(sensor, robot.COLOR_IS_RED);
        }
        return false;
    }

    public boolean colorIs(byte color)
    {
        if (color == robot.COLOR_IS_BOTH) {
            return (colorIs(robot.LEFT_COLOR, robot.COLOR_IS_RED) && colorIs(robot.RIGHT_COLOR, robot.COLOR_IS_BLUE)) ||
                    (colorIs(robot.RIGHT_COLOR, robot.COLOR_IS_RED) && colorIs(robot.LEFT_COLOR, robot.COLOR_IS_BLUE));
        } else if (color == robot.COLOR_IS_RED) {
            return !colorIs(robot.COLOR_IS_BOTH) &&
                    (colorIs(robot.LEFT_COLOR, robot.COLOR_IS_RED) || colorIs(robot.RIGHT_COLOR, robot.COLOR_IS_RED));
        } else if (color == robot.COLOR_IS_BLUE) {
            return !colorIs(robot.COLOR_IS_BOTH) &&
                    (colorIs(robot.LEFT_COLOR, robot.COLOR_IS_BLUE) || colorIs(robot.RIGHT_COLOR, robot.COLOR_IS_BLUE));
        } else if (color == robot.COLOR_IS_NONE) {
            return !colorIs(robot.COLOR_IS_BOTH) && !colorIs(robot.COLOR_IS_RED) && !colorIs(robot.COLOR_IS_BLUE);
        }
        return false;
    }
}