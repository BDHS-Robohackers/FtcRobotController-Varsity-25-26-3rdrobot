package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

@Autonomous(name="Blue Front 6", group="Autonomous")
public class BlueAuto3 extends LinearOpMode {

    private ElapsedTime time = new ElapsedTime();
    private Robot robot;

    private float FORWARD_TIME = AutonomousConfig.FORWARD_TIME;
    private float FORWARD_SPEED = AutonomousConfig.FORWARD_SPEED;

    private float STRAFE_TIME = AutonomousConfig.STRAFE_TIME;
    private float STRAFE_SPEED = AutonomousConfig.STRAFE_SPEED;

    private double SHOOT_TIME = 0.4;

    class TimedAction {
        double duration;
        Consumer<Void> action;

        TimedAction(double duration, Consumer<Void> action) {
            this.duration = duration;
            this.action = action;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        time = new ElapsedTime();
        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addData("Autonomous", "Ready");
        telemetry.update();

        waitForStart();
        time.reset();

        //
        // Build your action sequence here
        //
        List<TimedAction> plan = new ArrayList<>();
        // spin up
        plan.add(new TimedAction(5, v -> {
            robot.updateFlywheelMotors(-0.75);
            robot.updateDriveMotors(0,0,0);
        }));
        // Strafe
        plan.add(new TimedAction(1.5, v ->
                robot.updateDriveMotors(0,0.5,0)
        ));
        // stop
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0,0,0)
        ));
        // rotate
        plan.add(new TimedAction(0.5, v ->
                robot.updateDriveMotors(-0.45,0,0)
        ));
        // stop
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0,0,0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(1)
        ));

        plan.add(new TimedAction(0.25, v -> {
            robot.updateFlyFeedMotor(0);
            robot.updateIntakeMotors(0);
            robot.updateDriveMotors(0,0,0);
        }));

        // rotate
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0.35,0,0)
        ));
        // stop
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0,0,0)
        ));
        // strafe
        plan.add(new TimedAction(0.5, v ->
                robot.updateDriveMotors(0,0.8,0)
        ));
        // stop and start intake
        plan.add(new TimedAction(0.25, v -> {
            robot.updateDriveMotors(0, 0, 0);
            robot.updateIntakeMotors(1);
        }));
        // start forward
        plan.add(new TimedAction(1.75, v -> {
            robot.updateDriveMotors(0,0,-0.5);
        }));
        // stop but keep intake on
        plan.add(new TimedAction(0.25, v -> {
            robot.updateDriveMotors(0, 0, 0);
            robot.updateIntakeMotors(1);
        }));
        // stop intake and start backwards
        plan.add(new TimedAction(1.25, v -> {
            robot.updateDriveMotors(0, 0, 0.50);
            robot.updateIntakeMotors(0);
        }));
        // stop
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0,0,0)
        ));
        // start strafing back
        plan.add(new TimedAction(0.5, v ->
                robot.updateDriveMotors(0,-0.6,0)
        ));
        // stop
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0,0,0)
        ));
        // start rotating back
        //plan.add(new TimedAction(0.25, v ->
        //        robot.updateDriveMotors(-0.35,0,0)
        //));
        // stop
        plan.add(new TimedAction(0.25, v ->
                robot.updateDriveMotors(0,0,0)
        ));
        // shoot the balls we picked up
        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateIntakeMotors(0)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(1)
        ));

        plan.add(new TimedAction(SHOOT_TIME, v ->
                robot.updateFlyFeedMotor(0)
        ));

        // final safety stop
        plan.add(new TimedAction(Double.POSITIVE_INFINITY, v -> {
            robot.updateFlyFeedMotor(0);
            robot.updateDriveMotors(0,0,0);
            robot.updateFlywheelMotors(0);
            robot.updateIntakeMotors(0);
        }));

        //
        // Execution engine
        //
        int current = 0;
        double actionStart = time.seconds();

        while (opModeIsActive()) {
            double now = time.seconds();
            double elapsed = now - actionStart;

            if (current < plan.size()) {
                TimedAction step = plan.get(current);

                if (elapsed >= step.duration) {
                    current++;
                    actionStart = now;
                    if (current < plan.size()) {
                        plan.get(current).action.accept(null);
                    }
                } else {
                    step.action.accept(null);
                }
            }

            telemetry.addData("Step", current);
            telemetry.addData("Time", now);
            telemetry.update();
        }
    }
}
