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

    // flywheel RPM (adjust to your bot)
    private static final double FLYWHEEL_RPM = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addLine("AUTO READY (ENCODERS)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // set target flywheel RPM
        robot.setFlywheelRPM(FLYWHEEL_RPM);

        // Build action list
        List<Runnable> plan = new ArrayList<>();

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
        plan.add(robot::intakeStart);
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, SPEED));
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));
        plan.add(robot::intakeStop);

        // === STRAFE BACK ===
        plan.add(() -> robot.strafeEncoder(STRAFE_BACK, SPEED));

        // === SHOOT SECOND 3 ===
        plan.add(this::shootOne);
        plan.add(this::shootOne);
        plan.add(this::shootOne);
        plan.add(() -> robot.feedStop());

        // === shutdown ===
        plan.add(() -> {
            robot.stopFlywheel();
            robot.updateDriveMotors(0,0,0);
            robot.intakeStop();
            robot.feedStop();
        });

        // run actions sequentially while updating flywheel continuously
        for (Runnable action : plan) {
            if (!opModeIsActive()) break;

            // Start action in a separate thread style loop
            long startTime = System.currentTimeMillis();
            action.run();

            // During action, keep updating flywheel and telemetry
            while (opModeIsActive() && System.currentTimeMillis() - startTime < 50) {
                robot.updateFlywheelControl();
                telemetry.addData("Target RPM", FLYWHEEL_RPM);
                telemetry.addData("Current RPM", robot.getCurrentRPM());
                telemetry.update();
                sleep(20);
            }
        }

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    /** Shoots one ring cleanly while flywheel keeps spinning */
    private void shootOne() {
        long startTime;

        robot.intakeStart();
        startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 250) {
            robot.updateFlywheelControl();
            telemetry.addData("Target RPM", FLYWHEEL_RPM);
            telemetry.addData("Current RPM", robot.getCurrentRPM());
            telemetry.update();
            sleep(20);
        }
        robot.intakeStop();

        sleep(120); // delay for flywheel to keep spinning

        robot.feedStart();
        startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 300) {
            robot.updateFlywheelControl();
            telemetry.addData("Target RPM", FLYWHEEL_RPM);
            telemetry.addData("Current RPM", robot.getCurrentRPM());
            telemetry.update();
            sleep(20);
        }
        robot.feedStop();

        startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 150) {
            robot.updateFlywheelControl();
            telemetry.addData("Target RPM", FLYWHEEL_RPM);
            telemetry.addData("Current RPM", robot.getCurrentRPM());
            telemetry.update();
            sleep(20);
        }
    }
}
