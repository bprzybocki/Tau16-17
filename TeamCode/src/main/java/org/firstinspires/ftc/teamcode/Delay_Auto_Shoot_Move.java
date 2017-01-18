package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Benjamin on 10/19/2016.
 */

@Autonomous(name="Tau: Delay Auto Shoot Move", group="Tau")
//@Disabled
public class Delay_Auto_Shoot_Move extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV  = 1440;
    static final double DRIVE_GEAR_REDUCTION  = 1.0 / 3;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED           = 0.6;
    static final double TURN_SPEED            = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();



/*
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        waitForStart();

        /*leftSetPower(-0.5);
        rightSetPower(-0.5);

        sleepTau(1400);

        leftSetPower(0);
        rightSetPower(0);

        robot.flywheelMotorL.setPower(robot.FLYWHEEL_PWR);
        robot.flywheelMotorR.setPower(robot.FLYWHEEL_PWR);

        sleepTau(100);

        leftSetPower(0.5);
        rightSetPower(0.5);

        sleepTau(400);

        robot.flyWheelPiston.setPosition(robot.PISTON_UP);

        sleepTau(1000);

        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN); //ball 1 is launched

        sleepTau(2000); //temp

        robot.flyWheelPiston.setPosition(robot.PISTON_UP);

        sleepTau(1000);

        robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);

        robot.flywheelMotorL.setPower(0);
        robot.flywheelMotorR.setPower(0);
        */
        //encoderDrive(0.5,10,10,5);
        sleepTau(20000);
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
        robot.flywheelMotorL.setPower(robot.FLYWHEEL_PWR);
        robot.flywheelMotorR.setPower(robot.FLYWHEEL_PWR);
        sleepTau(550);
        goStraightPower(-0.5);
        sleepTau(1000);
        goStraightPower(0);

        //


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
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.rightBackMotor.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy() &&
                            robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())) {

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

            //  sleep(250);   // optional pause after each move
        }
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
