package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by Benjamin on 10/5/2016.
 */

@TeleOp(name="Tau: Teleop", group="Tau")
@Disabled
public class Teleop extends OpMode {

    Hardware robot = new Hardware();

    boolean isPistonUp = false;
    boolean enable = true;
    //byte
    // buttonPushingDirection = 1; // also known as trit :-) 0 = left | 1 = stopped | 2 = right
    final double CONST = 0.0; // example constant, but hardware related constants should be in Hardware class
    byte intakeState = 1; // 0 = backward | 1 = stopped | 2 = forward;
    double leftGP1 = 0, rightGP1 = 0,leftGP2 = 0;
    boolean isLeftStopped = true;
    boolean isRightStopped = true;
    boolean flywheelOn = false;
    double currentRPM = 0;

    // Controller layout
    //  Y
    // X  B
    //  A

    /*
        @Override
        public void runOpMode() throws InterruptedException {
            myInit();
            waitForStart();
            while (opModeIsActive()){
                myLoop();
            }
        }
    */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class dboes all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello ma dude");
        updateTelemetry(telemetry);
        //robot.buttonSensorL.enableLed(true);
        //robot.buttonSensorR.enableLed(true);
        robot.flywheelMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flywheelMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.flywheelMotorL.setMaxSpeed(robot.FLYWHEEL_SPD);
        //robot.flywheelMotorL.setMaxSpeed(robot.FLYWHEEL_SPD);
    }

    @Override
    public void init_loop() {
      /*  telemetry.addData("Red Left",robot.buttonSensorL.red());
        telemetry.addData("Blue Left",robot.buttonSensorL.blue());
        telemetry.addData("Green Left",robot.buttonSensorL.green());
        telemetry.addData("Alpha Left",robot.buttonSensorL.alpha());
        telemetry.addData("Red Right",robot.buttonSensorR.red());
        telemetry.addData("Blue Right",robot.buttonSensorR.blue());
        telemetry.addData("Green Right",robot.buttonSensorR.green());
        telemetry.addData("Alpha Right",robot.buttonSensorR.alpha());
        */
    }

    @Override
    public void start() {
        //robot.buttonSensorL.enableLed(false);
        //robot.buttonSensorL.enableLed(true);
        //robot.buttonSensorR.enableLed(false);
        //robot.buttonSensorR.enableLed(true);
    }

    @Override
    public void loop() {
       /* //enable = !enable;
        telemetry.addData("Red Left",robot.buttonSensorL.red());
        telemetry.addData("Blue Left",robot.buttonSensorL.blue());
        telemetry.addData("Green Left",robot.buttonSensorL.green());
        telemetry.addData("Alpha Left",robot.buttonSensorL.alpha());
        telemetry.addData("Red Right",robot.buttonSensorR.red());
        telemetry.addData("Blue Right",robot.buttonSensorR.blue());
        telemetry.addData("Green Right",robot.buttonSensorR.green());
        telemetry.addData("Alpha Right",robot.buttonSensorR.alpha());
        //telemetry.addData("RPM L",robot.flywheelMotorL.getMaxSpeed());
        //telemetry.addData("RPM R",robot.flywheelMotorR.getMaxSpeed());
*/
        telemetry.addData("Flywheel SPD", robot.FLYWHEEL_PWR);
        telemetry.addData("IntakeX", robot.intakeMotor.getCurrentPosition());
        telemetry.addData("CurrentRPM", currentRPM);
        telemetry.addData("Flywheel L", robot.flywheelMotorL.getMaxSpeed());
        telemetry.addData("Flywheel R", robot.flywheelMotorR.getMaxSpeed());
        telemetry.update();

        leftGP1 = -gamepad1.left_stick_y;

        rightGP1 = -gamepad1.right_stick_y;
        leftGP2 = -gamepad2.left_stick_y;

        if(Math.abs(leftGP1) < 0.05) {
            leftGP1 = 0.0;
        }
        if(Math.abs(leftGP2) < 0.05) {
            leftGP2 = 0.0;
        }
        if(Math.abs(rightGP1) < 0.05) {
            rightGP1 = 0.0;
        }
        if (gamepad1.left_trigger > 0.0) {
            rightGP1 /= 4;
            leftGP1 /= 4;
        }
/*
        if (leftGP1 == 0.0 && !isLeftStopped) {
            robot.leftFrontMotor.setPower(0.1);
            robot.leftBackMotor.setPower(0.1);
            sleepTau(100);
            isLeftStopped = true;
        } else if (Math.abs(leftGP1) > 0.1 && isLeftStopped) {
            isLeftStopped = false;
        }
        if (rightGP1 == 0.0 && !isRightStopped) {
            robot.rightFrontMotor.setPower(0.1);
            robot.rightBackMotor.setPower(0.1);
            sleepTau(100);
            isRightStopped = true;
        } else if (Math.abs(rightGP1) > 0.1 && isRightStopped) {
            isRightStopped = false;
        }
*/
        robot.leftMotor.setPower(0.35*leftGP1); //0.45
        robot.rightMotor.setPower(0.35*rightGP1);

        if (gamepad2.right_trigger > 0.0) {
            robot.flywheelMotorL.setPower(robot.FLYWHEEL_PWR);
            robot.flywheelMotorR.setPower(robot.FLYWHEEL_PWR);
        } else {
            robot.flywheelMotorR.setPower(0.0);
            robot.flywheelMotorL.setPower(0.0);
        }

        if (gamepad2.x) {
            robot.flyWheelPiston.setPosition(robot.PISTON_UP);
        }
        else {
            robot.flyWheelPiston.setPosition(robot.PISTON_DOWN);
        }

        if (gamepad1.x) {
            robot.buttonMotor.setPower(1.0);
        }
        else if (gamepad1.b) {
            robot.buttonMotor.setPower(-1.0);
        }
        else {
            robot.buttonMotor.setPower(0.0);
        }

        if (gamepad2.right_bumper)
        {
            telemetry.addData("Debug:", "Right Bump");
            robot.intakeMotor.setPower(0.0);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.intakeMotor.setPower(1.0);
        }
        else if (gamepad2.left_bumper)
        {
            telemetry.addData("Debug:", "Left Bump");
            robot.intakeMotor.setPower(0.0);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.intakeMotor.setPower(-1.0);
        }
        else if (gamepad2.a)
        {
            telemetry.addData("Debug:", "A");
            robot.intakeMotor.setPower(0.0);
        }

    }

    @Override
    public void stop() {
    }

    public void sleepTau(long millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
