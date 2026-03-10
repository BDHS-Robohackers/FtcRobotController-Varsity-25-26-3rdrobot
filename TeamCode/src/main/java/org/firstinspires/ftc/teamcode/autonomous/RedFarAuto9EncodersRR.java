package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="brett favre 9 (RR)", group="Autonomous")
public class RedFarAuto9EncodersRR extends LinearOpMode {

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
            return new startIntake();
        }
        public class stopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(0);
                return false;
            }
        }
        public Action stopIntake() {
            return new stopIntake();
        }
        public class reverseIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(-1);
                return false;
            }
        }
        public Action reverseIntake() {
            return new reverseIntake();
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
            return new stopFlywheel();
        }
        public class flywheelClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(1120);
                return false;
            }
        }
        public Action flywheelClose() {
            return new flywheelClose();
        }
        public class flywheelMedium implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(1200);
                return false;
            }
        }
        public Action flywheelMedium() {
            return new flywheelMedium();
        }
        public class flywheelFar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                flywheel.setVelocity(1430);
                return false;
            }
        }
        public Action flywheelFar() {
            return new flywheelFar();
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
            return new startFrontIntake();
        }
        public class stopFrontIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                frontIntake.setPower(0);
                return false;
            }
        }
        public Action stopFrontIntake() {
            return new stopFrontIntake();
        }
        public class reverseFrontIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                frontIntake.setPower(-1);
                return false;
            }
        }
        public Action reverseFrontIntake() {
            return new reverseFrontIntake();
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
        public Action shootOne() { return new shootOne();}
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
        public Action loadOne() { return new loadOne();}
        public class shootThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(0.75);
                feedFly.setPower(1);
                sleep(1500);
                intake.setPower(0);
                feedFly.setPower(0);
                return false;
            }
        }
        public Action shootThree() { return new shootThree();}
        public class shootThreeShort implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                intake.setPower(0.80);
                feedFly.setPower(1);
                sleep(1500);
                intake.setPower(0);
                feedFly.setPower(0);
                return false;
            }
        }
        public Action shootThreeShort() { return new shootThreeShort();}
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(60, 14, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        intake intake = new intake(hardwareMap);
        intakeSystem intakeSystem = new intakeSystem(hardwareMap);
        flywheel flywheel = new flywheel(hardwareMap);

        TrajectoryActionBuilder driveTo1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(48,12),Math.toRadians(150));


        TrajectoryActionBuilder driveToGet2 = drive.actionBuilder(new Pose2d(48,12,(Math.toRadians(150))))
                //.turnTo(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(30,12),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(30,60),Math.toRadians(90));
                //.waitSeconds(0.2);

        TrajectoryActionBuilder driveTo2 = drive.actionBuilder(new Pose2d(30,60,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48,12),Math.toRadians(165));

        TrajectoryActionBuilder driveToGet3 = drive.actionBuilder(new Pose2d(48,12,(Math.toRadians(165))))
                .strafeToLinearHeading(new Vector2d(63,67),Math.toRadians(80));
                //.strafeToLinearHeading(new Vector2d(16,60),Math.toRadians(90));
                //.waitSeconds(0.2);

        TrajectoryActionBuilder driveTo3 = drive.actionBuilder(new Pose2d(63,67,(Math.toRadians(80))))
                .strafeToLinearHeading(new Vector2d(48,12),Math.toRadians(165));

        TrajectoryActionBuilder driveToGet4 = drive.actionBuilder(new Pose2d(48,12,(Math.toRadians(165))))
                .strafeToLinearHeading(new Vector2d(53,67),Math.toRadians(80));
                //.strafeToLinearHeading(new Vector2d(16,60),Math.toRadians(90));
                //.waitSeconds(0.2);

        TrajectoryActionBuilder driveTo4 = drive.actionBuilder(new Pose2d(53,67,(Math.toRadians(80))))
                .strafeToLinearHeading(new Vector2d(47,12),Math.toRadians(165));


        TrajectoryActionBuilder driveToEnd = drive.actionBuilder(new Pose2d(47,12,Math.toRadians(165)))
                .strafeToLinearHeading(new Vector2d(54,36),Math.toRadians(-90));

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        flywheel.flywheelFar(),
                        new SleepAction(2.15),
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

                        intake.startIntake(),
                        driveToGet4.build(),
                        intake.stopIntake(),

                        driveTo4.build(),
                        intakeSystem.shootThreeShort(),

                        driveToEnd.build()

                )
        );
    }
}