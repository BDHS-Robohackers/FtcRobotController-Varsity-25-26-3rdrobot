package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Red Front 6 (ENCODERS V2)", group="Autonomous")
public class RedAuto6EncodersRev2 extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after real testing)
    private static final double FLYWHEEL_POWER = 0.80;
    private static final double STRAFE_1 = -25;
    private static final double TURN_1 = 4;
    private static final double TURN_2 = 2;
    private static final double FWD_INTAKE = 46;
    private static final double BACK_UP = -10;
    private static final double STRAFE_BACK = 14;

    private static final double SPEED = 0.85;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addLine("AUTO READY (ENCODERS)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build action list
        List<Runnable> plan = new ArrayList<>();

        // Spin up flywheel
        plan.add(this::spinUp);

        // === STRAFE RIGHT ===
        plan.add(() -> robot.strafeEncoder(-STRAFE_1, SPEED));

        // === TURN LEFT ===
        plan.add(() -> robot.turnEncoder(-TURN_1, SPEED));

        // === SHOOT 3 RINGS ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);

        plan.add(() -> robot.feedStop());

        // === STRAFE + MOVE TO BALLS ===
        plan.add(() -> robot.turnEncoder(TURN_2, SPEED)); // undo turn
        plan.add(() -> robot.strafeEncoder(26, SPEED));
        plan.add(() -> robot.turnEncoder(-2, SPEED)); // turn
        plan.add(() -> robot.updateIntakeMotors(1));  // intake on
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, 0.6)); // go forward
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));    // back up
        plan.add(() -> robot.updateIntakeMotors(0));

        // === STRAFE BACK ===
        plan.add(() -> robot.turnEncoder(-6,SPEED));

        // === SHOOT SECOND 3 ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(() -> robot.feedStop());

        // === shutdown ===
        plan.add(() -> {
            robot.updateFlywheelMotors(0);
            robot.updateDriveMotors(0,0,0);
            robot.updateIntakeMotors(0);
            robot.updateFlyFeedMotor(0);
        });

        // run actions sequentially while updating flywheel continuously
        for (Runnable action : plan) {
            if (!opModeIsActive()) break;
            action.run();
        }

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    /** Shoots one ring cleanly while flywheel keeps spinning */
    private void loadOne() {
        robot.updateIntakeMotors(1);
        sleep(500);
        robot.updateIntakeMotors(0);
        sleep(250);
    }
    private void shootOne() {
        sleep(250);
        robot.updateFlyFeedMotor(1);
        sleep(500);
        robot.updateFlyFeedMotor(0);
        sleep(500);
    }

    private void spinUp() {
        robot.updateFlywheelMotors(-FLYWHEEL_POWER);
        sleep(4500);
    }
}
