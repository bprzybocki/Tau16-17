package org.firstinspires.ftc.teamcode;

/**
 * Created by Erin on 2/6/2017.
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

@Autonomous(name = "Tau: Auto Test", group = "Tau")
//@Disabled
public class Auto_Test extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();


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

        telemetry.addData("Debug", robot.leftMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        telemetry.addData("Debug", "Running");
        telemetry.update();

        //DriveStraightAbsolute(0.25,5.0,0);



        boolean change = true;
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

        while (robot.leftMotor.getCurrentPosition()*LEFT_POLARITY < target_count) {
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
            return -heading;
        else
            return 360 - heading;
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
}
