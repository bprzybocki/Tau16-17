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
        telemetry.addData("Status", "Calibrating... Don't press start!");
        robot.init(hardwareMap);
        robot.mrGyro.calibrate();
        //while (robot.mrGyro.isCalibrating()) {}
        robot.mrGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.rightFrontMotor.setPower(0.5);
        robot.leftFrontMotor.setPower(0.5);
        robot.sleepTau(5000);

        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.sleepTau(1000);

        DriveStraightAbsolute(0.25,5.0,0);
    }

    //
    // DriveStraightAbsolute() - move the robot forward for the given number
    //                           of tiles at the given speed. The "straightness"
    //                           is based upon the given heading (negative CW
    //                           and positive CCW).
    //
    public void DriveStraightAbsolute(double speed, double tiles, int targetHeading)
    {
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int target_count = (int)(tiles*6000);
        int error;
        double ERROR_ADJUSTMENT = 0.2;

        while (robot.leftFrontMotor.getCurrentPosition() < target_count) {
            error = targetHeading - getHeading(); //positive error means need to go counterclockwise
            robot.leftFrontMotor.setPower(speed - error*ERROR_ADJUSTMENT);
            robot.rightFrontMotor.setPower(speed + error*ERROR_ADJUSTMENT);
            telemetry.addData("Gyro Heading Raw", robot.mrGyro.getHeading());
            telemetry.addData("Gyro Error", error);
            telemetry.addData("Left Ticks", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("Right Ticks", robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();
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
            return heading;
        else
            return -(360 - heading);
    }
}
