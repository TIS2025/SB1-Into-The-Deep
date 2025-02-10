package org.firstinspires.ftc.teamcode.auto.Sample;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AUTOINIT;
import org.firstinspires.ftc.teamcode.Sequences.INIT;
import org.firstinspires.ftc.teamcode.Sequences.sampleDrop;
import org.firstinspires.ftc.teamcode.Sequences.sampleDropPose;
import org.firstinspires.ftc.teamcode.Sequences.sampleIntake;
import org.firstinspires.ftc.teamcode.Sequences.samplePreTransfer;
import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

import java.util.Arrays;


@Config
@Autonomous (name = "Sample(2+3)",group = "Autonomous")
public class Sample5 extends LinearOpMode {

    VelConstraint speedup80 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

    private RobotHardware robot = RobotHardware.getInstance();

    private intake_ss intake_ss;
    private outake_ss outake_ss;
    private elevator_ss elevator_ss;

    private volatile boolean beamInterrupted; // Shared variable for sensor monitoring

    public int state = 0;

    public static boolean sample1 = true;
    public static boolean sample2 = true;
    public static boolean sample3 = true;
    public static boolean sample4 = true;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        intake_ss = new intake_ss(robot);
        outake_ss = new outake_ss(robot);
        elevator_ss = new elevator_ss(robot);

        Pose2d intialPose = new Pose2d(-35, -60, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);

        TrajectoryActionBuilder trajectoryAction0 = drive.actionBuilder(intialPose)
                //TODO - Preload Sample Droping
                .stopAndAdd(() -> new sampleDropPose(intake_ss,outake_ss, elevator_ss,1))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-59, -57.5), Math.toRadians(15))
                .stopAndAdd(() -> new sampleDrop(outake_ss))

                //TODO 2nd Preload Picking
                .afterTime(1, () -> new sampleIntake(intake_ss, outake_ss, elevator_ss, "Preloadd2"))
                .strafeToLinearHeading(new Vector2d(17.5, -58), Math.toRadians(0));

        //TODO - 2nd Preload Droppong
        TrajectoryActionBuilder trajectoryAction1 = trajectoryAction0.endTrajectory().fresh()
                .stopAndAdd(samplePreTransfer.samplePreTransfer(intake_ss, outake_ss, elevator_ss))
                .afterTime(1.6, () -> new sampleDropPose(intake_ss, outake_ss, elevator_ss,2))
                .strafeToLinearHeading(new Vector2d(-64.5, -53), Math.toRadians(65))
                .waitSeconds(0.5)
                .stopAndAdd(() -> new sampleDrop(outake_ss));

        //TODO 1nd Sample Picking
        TrajectoryActionBuilder trajectoryAction2 = trajectoryAction1.endTrajectory().fresh()
                .afterTime(0.01, () -> new sampleIntake(intake_ss, outake_ss, elevator_ss, 2))
                .strafeToLinearHeading(new Vector2d(-52, -46.5), Math.toRadians(90));

        //TODO - 1nd Sample Droppong
        TrajectoryActionBuilder trajectoryAction3 = trajectoryAction2.endTrajectory().fresh()
                .stopAndAdd(() -> new sampleDropPose(intake_ss, outake_ss, elevator_ss))
                .afterTime(0.01,()-> new sampleIntake(intake_ss,elevator_ss))
                .strafeToLinearHeading(new Vector2d(-67, -55), Math.toRadians(76))
                .waitSeconds(0.25)
                .stopAndAdd(() -> new sampleDrop(outake_ss));

        //TODO 2nd Sample Picking
        TrajectoryActionBuilder trajectoryAction4 = trajectoryAction3.endTrajectory().fresh()
                .afterTime(0.01, () -> new sampleIntake(intake_ss, outake_ss, elevator_ss, 3))
                .strafeToLinearHeading(new Vector2d(-62, -48.5), Math.toRadians(90));
//                .waitSeconds(0.5);

        //TODO - 2nd Sample Droppong
        TrajectoryActionBuilder trajectoryAction5 = trajectoryAction4.endTrajectory().fresh()
                .stopAndAdd(() -> new sampleDropPose(intake_ss, outake_ss, elevator_ss))
                .waitSeconds(0.7)
                .afterTime(0.01,()-> new sampleIntake(intake_ss,elevator_ss))
                .strafeToLinearHeading(new Vector2d(-66.5, -54.5), Math.toRadians(73))
                .stopAndAdd(() -> new sampleDrop(outake_ss));

        //TODO 3rd Sample Picking
        TrajectoryActionBuilder trajectoryAction6 = trajectoryAction5.endTrajectory().fresh()
                .afterTime(0.01, () -> new sampleIntake(intake_ss, outake_ss, elevator_ss, 3))
                .strafeToLinearHeading(new Vector2d(-59.5, -43.5), Math.toRadians(120));
//                .waitSeconds(0.5);

        //TODO - 3rd Sample Droppong
        TrajectoryActionBuilder trajectoryAction7 = trajectoryAction6.endTrajectory().fresh()
                .stopAndAdd(() -> new sampleDropPose(intake_ss, outake_ss, elevator_ss))
//                .afterTime(0.01,()-> new sampleIntake(intake_ss,elevator_ss))
                .strafeToLinearHeading(new Vector2d(-64.5, -56.5), Math.toRadians(70))
                .stopAndAdd(() -> new sampleDrop(outake_ss));

        //TODO - Parking
        TrajectoryActionBuilder trajectoryAction8 = trajectoryAction7.endTrajectory().fresh()
                .afterTime(0.1,()->new INIT(intake_ss,outake_ss,elevator_ss,"INIT"))
//                .strafeToLinearHeading(new Vector2d(-38, -5.5), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-22, -4,Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(2);


        final NormalizedRGBA[] rgba = new NormalizedRGBA[1];
        final float[][] hsv = new float[1][1];
        final double[] distance = new double[1];
        robot.colorSensor.setGain((float) globals.gain);

        if (opModeInInit()) {
            new AUTOINIT(intake_ss, outake_ss, elevator_ss);
            sample1 = true;
            sample2 = true;
            sample3 = true;
            sample4 = true;
        }

        waitForStart();

        // Thread for continuously checking the beam breaker and color sensor
        Thread sensorMonitor = new Thread(() -> {
            while (opModeIsActive()) {
                intake_ss.updateSholder();

                //TODO Color detection --> Do Not Change
                rgba[0] = robot.colorSensor.getNormalizedColors();
                distance[0] = robot.colorSensor.getDistance(DistanceUnit.MM);
                hsv[0] = rgbToHsv(rgba[0].red, rgba[0].green, rgba[0].blue);

                //TODO HSV YELLOW
                if (((hsv[0][0] < 80) && (hsv[0][0] > 55) && (hsv[0][2] > 0.95) && distance[0] < 30)) {
                    globals.intakeItem = 1;
                } else {
                    globals.intakeItem = 0;
                }

                telemetry.addData("Current Pose", drive.pose);
                telemetry.addData("Extention Position", robot.extender.getCurrentPosition());
                telemetry.addData("Extention Current", robot.extender.getCurrent(CurrentUnit.AMPS));

                telemetry.update();

                sleep(10);  // Check sensors every 50ms to avoid overloading CPU
            }
        });

        //TODO *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

        sensorMonitor.start();
        //TODO - Preload Droping and 2nd Preload Sample Pick Pose
        if (state == 0) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction0.build()
                    )
            );
            state = 1;
        }

        //TODO - 2nd Preload Sample Picking
        timer.reset();
        timer.startTime();
        while (state == 1) {
//            timer.startTime();
            if (sample1) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(750, 0.8))
                ));
                sample1 = false;
            }

            if (globals.intakeItem == 1) {
                Actions.runBlocking(intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.OFF));
//                Actions.runBlocking(
//                        new SequentialAction(
//                                new InstantAction(() -> new samplePreTransfer(intake_ss, outake_ss, elevator_ss))
//                        )
//                );
                state = 2;
            } else if (timer.seconds() > 0.5 && timer.seconds() < 1.5 && state != 1) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(700, 0.8))
                ));
            } else if (timer.seconds() > 1.5) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(200, 1))
                ));
                state = 2;
                timer.reset();
            }
        }

        //TODO - 2nd Preload Sample Droping
        if (state == 2) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction1.build()
                    )
            );
            state = 3;
        }

        //TODO - 1st Sample Pick Pose
        if (state == 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction2.build()
                    )
            );
            state = 4;
        }

        timer.reset();
        timer.startTime();
        //TODO - 1st Sample Picking
        while (state == 4) {

            if (sample2) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(700, 0.8)) // 395
                ));
                sample2 = false;
            }

            if (globals.intakeItem == 1) {
                Actions.runBlocking(intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.OFF));
                Actions.runBlocking(
                        new SequentialAction(
                                samplePreTransfer.samplePreTransfer(intake_ss, outake_ss, elevator_ss)
                        )
                );
                state = 5;
            } else if (timer.seconds() > 0.5 && timer.seconds() < 3 && state != 1) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(570, 0.8))
                ));
            } else if (timer.seconds() > 4) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(200, 1))
                ));
                state = 6;
                timer.reset();
            }
        }

        //TODO - 1st Sample Droping
        if (state == 5) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction3.build()
                    )
            );
            state = 6;
        }

        //TODO - 2nd Sample Pick Pose
        if (state == 6) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction4.build()
                    )
            );
            state = 7;
        }

        timer.reset();
        timer.startTime();

        //TODO - 2nd Sample Picking
        while (state == 7) {
            if (sample3) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(650, 0.8)) // 700
                ));
                sample3 = false;
            }


            if (globals.intakeItem == 1) {
                Actions.runBlocking(intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.OFF));
                Actions.runBlocking(
                        new SequentialAction(
                                samplePreTransfer.samplePreTransfer(intake_ss, outake_ss, elevator_ss)
                        )
                );
                state = 8;
            } else if (timer.seconds() > 0.5 && timer.seconds() < 3 && state != 1) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(780, 0.65))
                ));
            } else if (timer.seconds() > 4) {
                state = 9;
            }
        }

        //TODO - 2nd Sample Drop Pose
        if (state == 8) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction5.build()
                    )
            );
            state = 9;
        }

        //TODO - 3rd Sample Pick Pose
        if (state == 9) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction6.build()
                    )
            );
            state = 10;
        }

        timer.reset();
        timer.startTime();

        //TODO - 3rd Sample Picking
        while (state == 10) {
            if (sample3) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(650, 0.8)) // 700
                ));
                sample3 = false;
            }


            if (globals.intakeItem == 1) {
                Actions.runBlocking(intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.OFF));
                Actions.runBlocking(
                        new SequentialAction(
                                samplePreTransfer.samplePreTransfer(intake_ss, outake_ss, elevator_ss)
                        )
                );
                state = 11;
            } else if (timer.seconds() > 0.5 && timer.seconds() < 3 && state != 1) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(() -> extender(780, 0.65))
                ));
            } else if (timer.seconds() > 4) {
                state = 12;
            }
        }

        //TODO - 3rd Sample Drop Pose
        if (state == 11) {
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction7.build()
                    )
            );
            state = 12;
        }

        //TODO - Parking
        if(state == 12){
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction8.build()
                    )
            );
            state = 13;
        }


    }

    public void extender(int targetPos, double pow) {
        robot.extender.setTargetPosition(targetPos);
        robot.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extender.setTargetPositionTolerance(10);
        robot.extender.setPower(pow);
    }

    public float[] rgbToHsv(float rNorm, float gNorm, float bNorm) {
        float[] hsv = new float[3];

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;
        // Value
        hsv[2] = max;

        // Saturation
        hsv[1] = max == 0 ? 0 : delta / max;

        // Hue
        if (delta == 0) {
            hsv[0] = 0;
        } else {
            if (max == rNorm) {
                hsv[0] = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (max == gNorm) {
                hsv[0] = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (max == bNorm) {
                hsv[0] = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }

        return hsv;
    }
}