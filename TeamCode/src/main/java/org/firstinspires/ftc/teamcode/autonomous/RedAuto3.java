package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Red Front 3", group="Autonomous")
public class RedAuto3 extends LinearOpMode {

    private ElapsedTime time = new ElapsedTime();
    private Robot robot;

    private float FORWARD_TIME = AutonomousConfig.FORWARD_TIME;
    private float FORWARD_SPEED = AutonomousConfig.FORWARD_SPEED;

    private float STRAFE_TIME = AutonomousConfig.STRAFE_TIME;
    private float STRAFE_SPEED = AutonomousConfig.STRAFE_SPEED;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors and anything else
        time = new ElapsedTime();
        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addData("Autonomous", "The robot will perform these actions:");
        telemetry.addData("- Idle", "1 second");
        telemetry.addData("- Move forward", FORWARD_TIME+" seconds");
        telemetry.addData("at power", FORWARD_SPEED);
        telemetry.addData("- Move left", STRAFE_TIME+" seconds");
        telemetry.addData("at power", STRAFE_SPEED);
        telemetry.addData("- Move backward", FORWARD_TIME+" seconds");
        telemetry.addData("at power", FORWARD_SPEED);
        telemetry.update();

        waitForStart();

        time.reset();
        while (opModeIsActive()) {
            if (time.seconds() < 2.5) {
                // Idle
                robot.updateFlywheelMotors(-0.75);
                robot.updateDriveMotors(0, 0, 0);
            } else if (time.seconds() < 1.5+1 + 1.5) {
                // Move Sideways
                robot.updateDriveMotors(0, -0.5, 0);
            } else if (time.seconds() < 2+1 + 1.5 + 1) {
                // Idle
                robot.updateDriveMotors(0, 0, 0);
            } else if (time.seconds() < 2+1 + 1.5 + 1 + 0.5) {
                // Rotate a bit
                robot.updateDriveMotors(0.6, 0, 0);
            } else if (time.seconds() < 2+1 + 1.5 + 1 + 0.5 + 1) {
                // Idle
                robot.updateDriveMotors(0, 0, 0);
            } else if (time.seconds() < 2+5.5) {
                robot.updateFlyFeedMotor(1);
            } else if (time.seconds() < 2+6) {
                robot.updateFlyFeedMotor(0);
            } else if (time.seconds() < 2+6.5) {
                robot.updateIntakeMotors(1);
            } else if (time.seconds() < 2+7.5) {
                robot.updateIntakeMotors(0);
            } else if (time.seconds() < 2+8) {
                robot.updateFlyFeedMotor(1);
            } else if (time.seconds() < 2+8.5) {
                robot.updateFlyFeedMotor(0);
            } else if (time.seconds() < 2+9) {
                robot.updateIntakeMotors(1);
            } else if (time.seconds() < 2+10) {
                robot.updateIntakeMotors(0);
            } else if (time.seconds() < 12+0.5) {
                robot.updateFlyFeedMotor(1);
            } else if (time.seconds() < 12.5 + 0.5) {
                robot.updateFlyFeedMotor(0);
                robot.updateIntakeMotors(0);
                robot.updateDriveMotors(-0.45, 0, 0);
            } else if (time.seconds() < 13 + 0.5) {
                robot.updateDriveMotors(0, -0.25, 0);
            } else if (time.seconds() < 13.5 + 0.25) {
                robot.updateDriveMotors(0, 0, 0);
            } else if (time.seconds() < 13.75 + 1.75) {
                robot.updateDriveMotors(0,0,-0.75);
                robot.updateIntakeMotors(1);
            } else {
                // Stop the robot
                robot.updateFlyFeedMotor(0);
                robot.updateDriveMotors(0, 0, 0);
                robot.updateFlywheelMotors(0);
                robot.updateIntakeMotors(0);
            }
        }
    }
}
