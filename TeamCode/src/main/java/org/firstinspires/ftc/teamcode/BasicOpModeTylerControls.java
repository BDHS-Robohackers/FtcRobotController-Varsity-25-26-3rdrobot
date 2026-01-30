package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Driver Op Mode (Tyler's Controls)", group = "Driver Op Mode")
public class BasicOpModeTylerControls extends LinearOpMode {

    /** @noinspection FieldMayBeFinal*/
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

    private Gamepad driverController;
    private Gamepad otherController;


    private double intakeControl = 0;
    private double frontIntakeControl = 1;
    private boolean isTheButtonPressed = false;
    private boolean isLBPressed = false;

    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;
    public static double TARGET_FLY_SPEED_THRESH = 60;
    public double flyCurrent = 0;
    public double currentFlywheelVelocity = 0;

    double P = 115;
    double F = 15;
    double targetFlywheelVelocity = 0;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);

    @Override
    public void runOpMode() {
        driverController = gamepad1;
        otherController = gamepad2;
        robot = new Robot();
        robot.initialize(hardwareMap);

        // Wait for the robot to start (driver presses PLAY).
        telemetry.addData("Status", "Initialized.");
        telemetry.addData("Controls", "Use the following controls:");
        telemetry.addData("Strafe Left", "Left Trigger");
        telemetry.addData("Strafe Right", "Right Trigger");
        telemetry.addData("Forward/Back", "LEFT Stick up/down");
        telemetry.addData("Rotate", "RIGHT Stick left/right");
        telemetry.addData("Flywheel On", "Push X to Switch on");
        telemetry.addData("Flywheel Off", "Push B to Switch off");
        telemetry.addData("Ethan Servo Control", "D-pad Up: Forward, D-pad Down: Reverse");
        telemetry.addData("Good luck!", "DON'T CRASH THE ROBOT PLS :)");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robot.fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.fly.setDirection(DcMotorEx.Direction.REVERSE);


        // Run until the end of the match (driver presses STOP or time runs out).
        while (opModeIsActive()) {
            robot.fly.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            updateDrive();
            updateFlywheel();
            updateIntake();
            telemetry.addData("Target Fly Vel : ",targetFlywheelVelocity);
            telemetry.addData("Current Fly Vel : ",currentFlywheelVelocity);
            telemetry.addData("Intake %",intakeControl*100);
            telemetry.addData("Yaw (rotate) %", yaw);
            telemetry.addData("Axial (FW/RV) %", axial);
            telemetry.addData("Lateral (Strafe) %", lateral);
            telemetry.addData("flywheel current : ",flyCurrent);
            // Update Ethan servo control based on D-pad input

            telemetry.update();
        }
    }

    // Update driving controls (tank drive or similar).
    private void updateDrive() {

        boolean isDrivingEnabled = !driverController.a;

        if (isDrivingEnabled) {
            axial = (1.0 * driverController.left_stick_y); // FWD/REV
            yaw = (1.0 * driverController.right_stick_x); // Rotate
            lateral = (0.6 * (driverController.left_trigger - driverController.right_trigger)); // Strafing
            robot.updateDriveMotors(axial, lateral, yaw);
        } else {
            axial = 0;
            lateral = 0;
            yaw = 0;
            robot.lockWheels();
        }


    }

    // Update flywheel motors based on X and B button presses
    private void updateFlywheel() {

        // variable control based on buttons pushed
        if (otherController.right_bumper) {
            targetFlywheelVelocity = 0;
        } else if (otherController.x) {
            targetFlywheelVelocity = 1160;
        } else if (otherController.a) {
            targetFlywheelVelocity = 1200;
        } else if (otherController.b) {
            targetFlywheelVelocity = 1500;
        } else if (otherController.start) {
            if (!isTheButtonPressed) {
                targetFlywheelVelocity += 20;
                isTheButtonPressed = true;
            }
        } else if (otherController.back) {
            if (!isTheButtonPressed) {
                targetFlywheelVelocity -= 20;
                isTheButtonPressed = true;
            }
        } else {
            isTheButtonPressed = false;
        }

        boolean isYPressed = otherController.y;
        currentFlywheelVelocity = -(robot.fly.getVelocity());
        flyCurrent = robot.fly.getCurrent(CurrentUnit.MILLIAMPS);
        boolean canShoot = ((currentFlywheelVelocity > (targetFlywheelVelocity - TARGET_FLY_SPEED_THRESH)) && (currentFlywheelVelocity < (targetFlywheelVelocity + TARGET_FLY_SPEED_THRESH)));

        if (targetFlywheelVelocity > 0 && isYPressed && (canShoot || otherController.right_trigger > 0.25)) {
            robot.updateFlyFeedMotor(1);
        } else {
            robot.updateFlyFeedMotor(0);
        }



        robot.fly.setVelocity(-targetFlywheelVelocity);
    }

    private void updateIntake() {
        if (otherController.dpad_up) {
            intakeControl = -1;
        } else if (otherController.dpad_down) {
            intakeControl = 1;
        } else {
            intakeControl = 0;
        }

        if (otherController.left_bumper) {
            if (!isLBPressed) {
                if (frontIntakeControl == 1) {
                    frontIntakeControl = -1;
                } else {
                    frontIntakeControl = 1;
                }
                isLBPressed = true;
            }
        } else {
            isLBPressed = false;
        }

        robot.updateIntakeMotors(intakeControl);
        robot.updateFrontIntakeMotors(frontIntakeControl);
    }

    // Update Ethan servo based on D-pad input

}