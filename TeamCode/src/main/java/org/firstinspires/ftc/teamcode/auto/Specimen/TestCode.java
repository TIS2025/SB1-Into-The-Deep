package org.firstinspires.ftc.teamcode.auto.Specimen;//// Kashif Auto Specimen
//
//package org.firstinspires.ftc.teamcode.AllAutos.Red;
//
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Sequences.Sample.SamplePickSeq;
//import org.firstinspires.ftc.teamcode.Sequences.Specimen.SpecimenReadyToDrop;
//import org.firstinspires.ftc.teamcode.Sequences.autoseq.AutoAfterSpecimenDrop;
//import org.firstinspires.ftc.teamcode.Sequences.autoseq.AutoSpecimenDropSeq;
//import org.firstinspires.ftc.teamcode.Sequences.autoseq.InitialInitSeq;
//import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.ElbowCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.IntakeRollerCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.ShoulderCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.SpecimenGripCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.ViperCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.WristCommand;
//import org.firstinspires.ftc.teamcode.commandbase.instantcommands.XExtensionCommand;
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.subsystem.ElevatorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystem.SpecimenSubsystem;
//
//import java.util.Arrays;
//
//@Config
//@Autonomous(group = "SPECIMEN AUTO")
//public class ARedSpecimenWithAfterTime extends LinearOpMode {
//    private RobotHardware robot = RobotHardware.getInstance();
//    //Subsystems
//    IntakeSubsystem intake = null;
//    SpecimenSubsystem specimen = null;
//    ElevatorSubsystem elevator = null;
//
//    //Drive
//    private MecanumDrive drive = null;
//    static Vector2d samplePick1 = new Vector2d(27, -42);//26, -43)
//
//
//    //TODO SPECIMEN PICK
//    public static double pickOffset = 4;
//    public static Vector2d specimenPick1 = new Vector2d(30.5, -66);
//    public static Vector2d specimenPick2 = new Vector2d(27 + 5, -66);
//    public static Vector2d specimenPick3 = new Vector2d(27 + 5, -66);
//
//    //TODO TESTING PROFILE ACCELERATION
//    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
//            new TranslationalVelConstraint(90)
////            new AngularVelConstraint(Math.PI / 2)
//    ));
//
//    AccelConstraint baseAccelConstraint2 = new ProfileAccelConstraint(-45, 90);
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap, telemetry);
//
//        intake = new IntakeSubsystem(robot);
//        specimen = new SpecimenSubsystem(robot);
//        elevator = new ElevatorSubsystem(robot);
//
//        Pose2d startPose = new Pose2d(17.5, -64, Math.toRadians(90));
//        drive = new MecanumDrive(hardwareMap, startPose);
//
//
//        //TODO ===============================================TRAJECTORIES =============================================================
//
//
//        Action trajectoryAction = drive.actionBuilder(drive.pose)
//                //TODO -SPECIMEN DROP
//
//                .afterTime(0.1, () -> new SpecimenReadyToDrop(specimen, 1))
//                .splineToConstantHeading(new Vector2d(5, -31), Math.toRadians(90))
//                //TODO PRELOAD DROP
//                .stopAndAdd(() -> new AutoSpecimenDropSeq(specimen, 0.2, 2))
//                .setReversed(true)
//                .afterTime(0.5, () -> new AutoAfterSpecimenDrop(specimen)) //add sleepfor wrist rotate
//
//                //TODO -GOTO SAMPLE PICK
//                //TODO PICK 1
////                .splineToLinearHeading(new Pose2d(35, -46, Math.toRadians(48)), Math.toRadians(90)) //35, -45
//                .afterTime(1.2,()->new SamplePickSeq(intake, 3))
//                .splineToLinearHeading(new Pose2d(36, -46, Math.toRadians(48)), Math.toRadians(35)) //34.5, -45.5
//                //PICKING SEQ
//                .waitSeconds(0.15)
//                .stopAndAdd(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK))
//                .stopAndAdd(() -> new IntakeRollerCommand(intake, IntakeSubsystem.IntakeRollerState.ON))
//                .waitSeconds(0.15)
//                .stopAndAdd(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
////                //Picked
//
//                //TODO 1st sample drop
//                .afterTime(0.6,()->new IntakeRollerCommand(intake, IntakeSubsystem.IntakeRollerState.RELEASE))
//                .strafeToLinearHeading(new Vector2d(34, -43.0001), Math.toRadians((-48)))//34, -43.0001
//
////                //TODO 2ND SAMPLE PICK
//                .afterTime(0.001,()->new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
//
//                .strafeToLinearHeading(new Vector2d(41.5, -43), Math.toRadians(48))//42.5, -43
//                //PICKING SEQ
//                .waitSeconds(0.15)
//                .stopAndAdd(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK))
//                .stopAndAdd(() -> new IntakeRollerCommand(intake, IntakeSubsystem.IntakeRollerState.ON))
//                .waitSeconds(0.15)
//                .stopAndAdd(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PRE_PICK))
////                //Picked
//
//                // TODO 2nd sample drop
//                .afterTime(0.6,()->new IntakeRollerCommand(intake, IntakeSubsystem.IntakeRollerState.RELEASE))
//                .strafeToLinearHeading(new Vector2d(40.00, -46), Math.toRadians((-48)))//new Vector2d(40.00, -44), Math.toRadians((-48))
//
////
////                //TODO 3 SAMPLE PICK
//                .strafeToLinearHeading(new Vector2d(52, -44), Math.toRadians(48))//45
//                .afterTime(0,()->new SamplePickSeq(intake, 3))
//                .strafeToLinearHeading(new Vector2d(55, -46), Math.toRadians(48))
//                .waitSeconds(0.15)
//                .stopAndAdd(() -> new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.SAMPLE_PICK))
//                .stopAndAdd(() -> new IntakeRollerCommand(intake, IntakeSubsystem.IntakeRollerState.ON))
//                .waitSeconds(0.15)
//                .stopAndAdd(() -> {
//                    new XExtensionCommand(intake, IntakeSubsystem.XExtensionState.HOME);
//                    robot.elbow.setPosition(0.601);
//                    new WristCommand(intake, IntakeSubsystem.WristState.RIGHT_DIAGONAL);
//                    robot.leftShoulder.setPosition(0.3606);
//                })
//
//                //TODO 3RD SAMPLE DROP
//                .afterTime(0.8, () -> {
//                    robot.viper.setPosition(0.25);//0.238
//
//                })
//                .afterTime(1, () -> {
//                    new IntakeRollerCommand(intake, IntakeSubsystem.IntakeRollerState.RELEASE);
//                })
//                .strafeToLinearHeading(new Vector2d(41, -67), Math.toRadians(90))
//                .stopAndAdd(() -> new SpecimenReadyToDrop(specimen))
//                .waitSeconds(0.1)
////                .waitSeconds(0.3)
////                .stopAndAdd(()-> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE))
////                //TODO DROP 1st SPECIMEN  DROP
//                .afterTime(0,()-> {new ViperCommand(intake, IntakeSubsystem.ViperState.INIT);
//                    new ShoulderCommand(intake, IntakeSubsystem.ShoulderState.INIT);
//                })
////                .afterTime(0.4,()->new SpecimenReadyToDrop(specimen))
//                .splineToLinearHeading(new Pose2d(1, -31.40, Math.toRadians(90)), Math.toRadians(135))
//                .stopAndAdd(() -> new AutoSpecimenDropSeq(specimen, 0.2, 2))
//                .setReversed(true)
//                .afterTime(0.5, () -> new AutoAfterSpecimenDrop(specimen))
//                .splineToLinearHeading(new Pose2d(33, -67.500001, Math.toRadians(90)), Math.toRadians(-45))
//                .stopAndAdd(() -> new SpecimenReadyToDrop(specimen))
//                .waitSeconds(0.1)
////                .waitSeconds(0.15)
////                .stopAndAdd(()-> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE))
////                .waitSeconds(0.15)
////                //TODO DROP 2nd SPECIMEN  DROP
////                .afterTime(0.4,()->new SpecimenReadyToDrop(specimen))
//                .splineToLinearHeading(new Pose2d(-2, -30.40, Math.toRadians(90)), Math.toRadians(135))
//                .stopAndAdd(() -> new AutoSpecimenDropSeq(specimen, 0.2, 2))
//                .setReversed(true)
//                .afterTime(0.5, () -> new AutoAfterSpecimenDrop(specimen))
//                .splineToLinearHeading(new Pose2d(33, -67.50001, Math.toRadians(90)), Math.toRadians(-45))
//                .stopAndAdd(() -> new SpecimenReadyToDrop(specimen))
//                .waitSeconds(0.1)
////                .waitSeconds(0.15)
////                .stopAndAdd(()-> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE))
////                .waitSeconds(0.15)
////                //TODO DROP 3rd SPECIMEN  DROP
////                .afterTime(0.4,()->new SpecimenReadyToDrop(specimen))
//                .splineToLinearHeading(new Pose2d(-3, -30.40, Math.toRadians(90)), Math.toRadians(135), baseVelConstraint, baseAccelConstraint2)
//                .stopAndAdd(() -> new AutoSpecimenDropSeq(specimen, 0.2, 2))
//                .setReversed(true)
//                .afterTime(0.4, () -> new AutoAfterSpecimenDrop(specimen))
//                .splineToLinearHeading(new Pose2d(33, -67.5001, Math.toRadians(90)), Math.toRadians(-45))
//                .stopAndAdd(() -> new SpecimenReadyToDrop(specimen))
//                .waitSeconds(0.1)
////                .waitSeconds(0.15)
////                .stopAndAdd(()-> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE))
////                .waitSeconds(0.15)
//
////                //TODO DROP 4 SPECIMEN DROP
////                .afterTime(0.4,()->new SpecimenReadyToDrop(specimen))
//                .splineToLinearHeading(new Pose2d(-4, -31.40, Math.toRadians(90)), Math.toRadians(135), baseVelConstraint, baseAccelConstraint2)
//                .stopAndAdd(() -> new AutoSpecimenDropSeq(specimen, 0.2, 2))
//                .setReversed(true)
//                .afterTime(0.4, () -> new AutoAfterSpecimenDrop(specimen))
//                //TODO PARK
//
//                .splineToLinearHeading(new Pose2d(33, -67.5000, Math.toRadians(90)), Math.toRadians(-45))
////                .waitSeconds(0.3)
////                .stopAndAdd(()-> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE))
////
////                //TODO DROP 5 SPECIMEN DROP
////                .afterTime(0,()->new SpecimenReadyToDrop(specimen,2))
////                .splineToLinearHeading(new Pose2d(-6, -31.40, Math.toRadians(90)), Math.toRadians(135), baseVelConstraint, baseAccelConstraint2)
////                .stopAndAdd(() -> new AutoSpecimenDropSeq(specimen, 0.2, 2))
////                .setReversed(true)
////                .afterTime(0.4, () -> new AutoAfterSpecimenDrop(specimen))
////                .splineToLinearHeading(new Pose2d(37.00, -66.5, Math.toRadians(90)), Math.toRadians(-45), baseVelConstraint, baseAccelConstraint2)
//
//                .build();
//
//
//        if (opModeInInit()) {
//            telemetry.addLine("ROBOT INIT MODE");
//            Actions.runBlocking(new SequentialAction(
//                    new InstantAction(() -> new InitialInitSeq(intake, specimen, elevator))
////                    new SleepAction(0.5),
////                    new InstantAction(() -> new SpecimenGripCommand(specimen, SpecimenSubsystem.SpecimenGripState.GRIP_CLOSE))
//            ));
//
//        }
//
//
//        waitForStart();
//
//        Actions.runBlocking(new SequentialAction(
//                trajectoryAction
//        ));
//        while (opModeIsActive()) {
//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//            telemetry.addData(" Navx heading (deg)", TwoDeadWheelLocalizer.robotHeading);
//            telemetry.update();
//        }
//
//
//    }
//}
//
///*
//wrist -0.436
//shoulder-0.3628
//elbow-0.27
//
// */
