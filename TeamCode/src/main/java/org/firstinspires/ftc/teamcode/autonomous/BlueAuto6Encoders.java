package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Blue Front 6 (ENCODERS - UPDATED)", group="Autonomous")
public class BlueAuto6Encoders extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after real testing)
    private static final double STRAFE_1 = -25;
    private static final double TURN_1 = 4;
    private static final double FWD_INTAKE = 28;
    private static final double BACK_UP = -20;
    private static final double STRAFE_BACK = 12;

    private static final double SPEED = 0.5;

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
        plan.add(() -> robot.updateFlywheelMotors(-0.75));

        // === STRAFE RIGHT ===
        plan.add(() -> robot.strafeEncoder(STRAFE_1, SPEED));

        // === TURN LEFT ===
        plan.add(() -> robot.turnEncoder(TURN_1, SPEED));

        // === SHOOT 3 RINGS ===
        plan.add(this::shootOne);
        plan.add(this::shootOne);
        plan.add(this::shootOne);

        plan.add(() -> robot.feedStop());

        // === STRAFE + MOVE TO BALLS ===
        plan.add(() -> robot.turnEncoder(-TURN_1, SPEED)); // undo turn
        plan.add(() -> robot.strafeEncoder(12, SPEED));
        plan.add(() -> robot.updateIntakeMotors(1));  // intake on
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, SPEED)); // go forward
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));    // back up
        plan.add(() -> robot.updateIntakeMotors(0));

        // === STRAFE BACK ===
        plan.add(() -> robot.strafeEncoder(STRAFE_BACK, SPEED));

        // === SHOOT SECOND 3 ===
        plan.add(this::shootOne);
        plan.add(this::shootOne);
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
    private void shootOne() {
        robot.updateIntakeMotors(1);
        sleep(350);
        robot.updateIntakeMotors(0);
        sleep(150);
        robot.updateFlyFeedMotor(1);
        sleep(350);
        robot.updateFlyFeedMotor(0);
        sleep(150);
    }
}
