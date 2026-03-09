package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Red Close 9 (RR)", group="Autonomous")
public class RedAuto9EncodersRR extends LinearOpMode {

    public class intake {
        private DcMotor intake;
        public intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class startIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(1);
                return false;
            }
        }
        public Action startIntake() {
            return new intake.startIntake();
        }
        public class stopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(0);
                return false;
            }
        }
        public Action stopIntake() {
            return new intake.stopIntake();
        }
        public class reverseIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(-1);
                return false;
            }
        }
        public Action reverseIntake() {
            return new intake.reverseIntake();
        }
    }
    public class flywheel {
        private DcMotorEx flywheel;
        public flywheel(HardwareMap hardwareMap){
            flywheel=hardwareMap.get(DcMotorEx.class,"fly");
            flywheel.setDirection(DcMotorEx.Direction.FORWARD);
            double P = 125;
            double I = 0;
            double D = 0;
            double F = 15;
            double targetFlywheelVelocity = 0;
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,I,D,F);
            flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
        public class stopFlywheel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(0);
                return false;
            }
        }
        public Action stopFlywheel() {
            return new flywheel.stopFlywheel();
        }
        public class flywheelClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(1120);
                return false;
            }
        }
        public Action flywheelClose() {
            return new flywheel.flywheelClose();
        }
        public class flywheelMedium implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(1200);
                return false;
            }
        }
        public Action flywheelMedium() {
            return new flywheel.flywheelMedium();
        }
        public class flywheelFar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(1440);
                return false;
            }
        }
        public Action flywheelFar() {
            return new flywheel.flywheelFar();
        }
    }
    public class intakeSystem {
        private DcMotor intake;
        private DcMotor frontIntake;
        private DcMotor feedFly;
        public intakeSystem(HardwareMap hardwareMap) {
            frontIntake = hardwareMap.get(DcMotor.class,"frontIntake");
            frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            feedFly = hardwareMap.get(DcMotor.class, "feedFly");
            feedFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            feedFly.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        public class startFrontIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                frontIntake.setPower(1);
                return false;
            }
        }
        public Action startFrontIntake() {
            return new intakeSystem.startFrontIntake();
        }
        public class stopFrontIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                frontIntake.setPower(0);
                return false;
            }
        }
        public Action stopFrontIntake() {
            return new intakeSystem.stopFrontIntake();
        }
        public class reverseFrontIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                frontIntake.setPower(-1);
                return false;
            }
        }
        public Action reverseFrontIntake() {
            return new intakeSystem.reverseFrontIntake();
        }
        public class shootOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                sleep(50);
                feedFly.setPower(1);
                sleep(250);
                feedFly.setPower(0);
                sleep(150);
                return false;
            }
        }
        public Action shootOne() { return new intakeSystem.shootOne();}
        public class loadOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(1);
                sleep(200);
                intake.setPower(0);
                sleep(50);
                return false;
            }
        }
        public Action loadOne() { return new intakeSystem.loadOne();}
        public class shootThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(0.75);
                feedFly.setPower(1);
                sleep(2000);
                intake.setPower(0);
                feedFly.setPower(0);
                return false;
            }
        }
        public Action shootThree() { return new intakeSystem.shootThree();}
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-42, 55.75, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        intake intake = new intake(hardwareMap);
        intakeSystem intakeSystem = new intakeSystem(hardwareMap);
        flywheel flywheel = new flywheel(hardwareMap);

        TrajectoryActionBuilder driveTo1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(135))
                .waitSeconds(0.2);



        TrajectoryActionBuilder driveToGet2 = drive.actionBuilder(new Pose2d(-12,12,(Math.toRadians(135))))
                //.turnTo(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,13),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,58),Math.toRadians(90));

        TrajectoryActionBuilder driveTo2 = drive.actionBuilder(new Pose2d(-12,58,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(140))
                .waitSeconds(0.2);

        TrajectoryActionBuilder driveToGet3 = drive.actionBuilder(new Pose2d(-12,12,(Math.toRadians(135))))
                .strafeToLinearHeading(new Vector2d(16,12),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(16,60),Math.toRadians(90));

        TrajectoryActionBuilder driveTo3 = drive.actionBuilder(new Pose2d(16,60,(Math.toRadians(90))))
                .strafeToLinearHeading(new Vector2d(0,40),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,56),Math.toRadians(90))
                .waitSeconds(0.75)
                .strafeToLinearHeading(new Vector2d(-14,12),Math.toRadians(145));

        TrajectoryActionBuilder driveToEnd = drive.actionBuilder(new Pose2d(-14,12,Math.toRadians(145)))
                .strafeToLinearHeading(new Vector2d(36,24),Math.toRadians(90));

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        flywheel.flywheelMedium(),
                        driveTo1.build(),

                        intakeSystem.shootThree(),
                        intakeSystem.startFrontIntake(),
                        intake.startIntake(),
                        driveToGet2.build(),
                        intake.stopIntake(),
                        driveTo2.build(),

                        intakeSystem.shootThree(),
                        intake.startIntake(),
                        driveToGet3.build(),
                        intake.stopIntake(),
                        driveTo3.build(),

                        intakeSystem.shootThree(),

                        driveToEnd.build()

                )
        );
    }
}