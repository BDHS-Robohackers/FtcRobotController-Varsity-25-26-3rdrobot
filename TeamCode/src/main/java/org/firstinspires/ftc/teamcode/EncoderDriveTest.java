package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Encoder Drive Test")
public class EncoderDriveTest extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addData("Status", "Robot initialized. Ready to test!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- DRIVE CONTROLS ---
        double axial = -gamepad1.left_stick_y;  // Forward/back
        double lateral = gamepad1.left_stick_x; // Strafe L/R
        double yaw = gamepad1.right_stick_x;    // Rotate

        robot.updateDriveMotors(axial, lateral, yaw);

        // --- FLYWHEEL + FEED + INTAKE CONTROLS ---
        if(gamepad1.right_bumper) robot.updateFlywheelMotors(1);
        else if(gamepad1.left_bumper) robot.updateFlywheelMotors(-1);
        else robot.updateFlywheelMotors(0);

        if(gamepad1.y) robot.updateFlyFeedMotor(1);
        else if(gamepad1.a) robot.updateFlyFeedMotor(-1);
        else robot.updateFlyFeedMotor(0);

        if(gamepad1.x) robot.updateIntakeMotors(1);
        else if(gamepad1.b) robot.updateIntakeMotors(-1);
        else robot.updateIntakeMotors(0);

        // --- TELEMETRY ENCODER INFO ---
        telemetry.addData("LF Encoder", robot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("RF Encoder", robot.rightFrontDrive.getCurrentPosition());
        telemetry.addData("LB Encoder", robot.leftBackDrive.getCurrentPosition());
        telemetry.addData("RB Encoder", robot.rightBackDrive.getCurrentPosition());

        telemetry.addData("Flywheel", robot.fly.getCurrentPosition());
        telemetry.addData("Feed", robot.feedFly.getCurrentPosition());
        telemetry.addData("Intake", robot.intake.getCurrentPosition());




        telemetry.addData("Drive Power", "Axial: %.2f Lateral: %.2f Yaw: %.2f", axial, lateral, yaw);
        telemetry.update();
    }
}