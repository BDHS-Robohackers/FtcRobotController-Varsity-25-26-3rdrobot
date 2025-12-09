package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Blue Front 6 (ENCODERS)", group="Autonomous")
public class BlueAuto6Encoders extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after testing)
    private static final double STRAFE_1 = 16;     // first strafe
    private static final double TURN_1 = 8;        // first rotate
    private static final double FWD_INTAKE = 28;   // move to pick up balls
    private static final double BACK_UP = -20;     // back up after intake
    private static final double STRAFE_BACK = -12; // side movement after backing up

    private static final double TURN_2 = -8;       // optional 2nd rotate

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
        plan.add(() -> robot.turnEncoder(TURN_1, SPEED * 0.9));

        // === SHOOT SEQUENCE (3 shots) ===
        plan.add(() -> shootOne());
        plan.add(() -> shootOne());
        plan.add(() -> shootOne());
        plan.add(() -> robot.updateFlyFeedMotor(0));

        // === STRAFE + MOVE TO BALLS ===
        plan.add(() -> robot.turnEncoder(-TURN_1, SPEED)); // undo turn
        plan.add(() -> robot.strafeEncoder(12, SPEED));
        plan.add(() -> robot.updateIntakeMotors(1));  // intake on
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, SPEED)); // go forward
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));    // back up
        plan.add(() -> robot.updateIntakeMotors(0));

        // === STRAFE BACK + REALIGN ===
        plan.add(() -> robot.strafeEncoder(STRAFE_BACK, SPEED));
        // optional second rotate back
        // plan.add(() -> robot.turnEncoder(TURN_2, SPEED));

        // === SHOOT SECOND SET ===
        plan.add(() -> shootOne());
        plan.add(() -> shootOne());
        plan.add(() -> shootOne());
        plan.add(() -> robot.updateFlyFeedMotor(0));

        // === shutdown ===
        plan.add(() -> {
            robot.updateFlywheelMotors(0);
            robot.updateDriveMotors(0,0,0);
            robot.updateIntakeMotors(0);
            robot.updateFlyFeedMotor(0);
        });

        // Execute actions sequentially
        for (Runnable action : plan) {
            if (!opModeIsActive()) break;
            action.run();
        }

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    // **Shoots a single ring cleanly**
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
