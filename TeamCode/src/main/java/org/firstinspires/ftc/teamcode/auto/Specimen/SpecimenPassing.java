package org.firstinspires.ftc.teamcode.auto.Specimen;// Navadiya Specimen Auto
//
//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.roadrunner.AccelConstraint;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.VelConstraint;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Hardware.Globals;
//import org.firstinspires.ftc.teamcode.Hardware.Resource;
//import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Sequence.SampleDropSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SamplePickSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SpecimenDropSeq;
//import org.firstinspires.ftc.teamcode.Sequence.SpecimenPickSeq;
//import org.firstinspires.ftc.teamcode.Subsystem.Elevator;
//import org.firstinspires.ftc.teamcode.Subsystem.Intake;
//import org.firstinspires.ftc.teamcode.Subsystem.Shoulder;
//
//import java.util.Arrays;
//
//@Autonomous(group = "#", name = "Specimen Push")
//public class SpecimenPush extends LinearOpMode {
//
//    private RobotHardware robot;
//    private Intake intake;
//    private Elevator elevator;
//    private Shoulder shoulder;
//    private MecanumDrive drive = null;
//
//    private Thread pidThread;
//    private Thread shoulderThread;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Globals.IS_AUTO = true;
//        robot = new RobotHardware();
//        robot.init(hardwareMap, telemetry);
//
//        intake = new Intake(robot);
//        elevator = new Elevator(robot);
//        shoulder = new Shoulder(robot);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(17.5, -64, Math.toRadians(-90)));
//
//        // Todo ==================================== VELO CONST ======================================================================
//        VelConstraint baseConst45 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(45)));
//        VelConstraint baseConst70 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(70)));
//        VelConstraint baseConst80 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));
//        VelConstraint baseConst90 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(90)));
//
//        // Todo ==================================== ACC CONST ======================================================================
//
//        AccelConstraint baseAccel50 = new ProfileAccelConstraint(-45.0, 80.0);
//
//        // Todo ==================================== MAIN SEQUENCE ======================================================================
//        Action autoSequence = drive.actionBuilder(new Pose2d(17.5, -64, Math.toRadians(-90)))
//
//                //TODO: PRELOAD
//
//                .afterTime(0.001, SpecimenDropSeq.AutoSpecimenPreDropAction(intake, elevator, shoulder))
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(4, -33.3), Math.toRadians(90))
//                .waitSeconds(0.1)
//                .stopAndAdd(() -> {
//                    elevator.ElevatorCommand(Elevator.ElevatorState.ELEVATOR_HOME);
//                    intake.ArmCommands(Intake.ArmState.SPECIMEN_DROP);
//                })
//                .waitSeconds(0.3)
//                .stopAndAdd(() -> intake.GripperCommands(Intake.GripperState.FULL_OPEN))
//                .waitSeconds(5)
//
//
//                //TODO: FIRST PUSH
//
////                .afterTime(0.2,SpecimenDropSeq.AutoSpecimenAfterDropDownAction(intake,elevator,shoulder))
//
//                .splineToLinearHeading(new Pose2d(new Vector2d(30, -40), Math.toRadians(-90)), Math.toRadians(0))
//
//                .splineToLinearHeading(new Pose2d(new Vector2d(46, -12), Math.toRadians(-90)), Math.toRadians(0))
//
//
//                .strafeToConstantHeading(new Vector2d(47.5, -45), baseConst90, baseAccel50)
//
//                //.setReversed(true)
//                //.splineToLinearHeading(new Pose2d(new Vector2d(42, -54), Math.toRadians(-90)),Math.toRadians(-90))
//
//
//                //TODO: SECOND PUSH
//
//
//                .splineToLinearHeading(new Pose2d(new Vector2d(57, -12), Math.toRadians(-90)), Math.toRadians(10))
//
//                .strafeToConstantHeading(new Vector2d(58.5, -45), baseConst90, baseAccel50)
//
//                //.setReversed(true)
//                //.splineToLinearHeading(new Pose2d(new Vector2d(52, -54), Math.toRadians(-90)),Math.toRadians(-90))
//
//                //TODO: THIRD PUSH
//
//
//                .splineToLinearHeading(new Pose2d(new Vector2d(66, -12), Math.toRadians(-90)), Math.toRadians(10))
//
//
//                //TODO: FIRST PICK
//
//
//                .afterTime(0.3, SpecimenPickSeq.AutoSecimenPrePickAction(intake, elevator, shoulder))
//                .strafeToConstantHeading(new Vector2d(64.5, -58))
//                .waitSeconds(0.2)
//                .stopAndAdd(SpecimenPickSeq.AutoSecimenPickAction(intake, elevator, shoulder))
//
//
//                .afterTime(0.6, SpecimenDropSeq.AutoSpecimenPreDropAction(intake, elevator, shoulder))
//
//
//                //.strafeToConstantHeading(new Vector2d(-2,-31),baseConst90,baseAccel50)
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(4.5, -44), Math.toRadians(90), baseConst90, baseAccel50)
//                .strafeToConstantHeading(new Vector2d(4.5, -33))
//                .waitSeconds(0.15)
//                .stopAndAdd(SpecimenDropSeq.AutoSpecimenDropAction(intake, elevator, shoulder))
//                .waitSeconds(0.2)
//                .stopAndAdd(() -> intake.GripperCommands(Intake.GripperState.FULL_OPEN))
//
//
//                //TODO: SECOND PICK
//
//
//                .afterTime(0.5, SpecimenPickSeq.AutoSecimenPrePickAction(intake, elevator, shoulder))
//
//                .strafeToConstantHeading(new Vector2d(35.5, -55))
//
//                //.splineToConstantHeading(new Vector2d(35.5, -61.4), Math.toRadians(-90))
//
//                .waitSeconds(0.2)
//                .stopAndAdd(SpecimenPickSeq.AutoSecimenPickAction(intake, elevator, shoulder))
//
//
//                // .strafeToConstantHeading(new Vector2d(4,-31))
//
//                .afterTime(0.6, SpecimenDropSeq.AutoSpecimenPreDropAction(intake, elevator, shoulder))
//
//                .strafeToConstantHeading(new Vector2d(1, -35.5))
//                // .setReversed(true)
//                // .splineToConstantHeading(new Vector2d(1, -31), Math.toRadians(90))
//                .waitSeconds(0.15)
//                .stopAndAdd(SpecimenDropSeq.AutoSpecimenDropAction(intake, elevator, shoulder))
//                .waitSeconds(0.2)
//                .stopAndAdd(() -> intake.GripperCommands(Intake.GripperState.FULL_OPEN))
//
//
//                //TODO: THIRD PICK
//
//                .afterTime(0.5, SpecimenPickSeq.AutoSecimenPrePickAction(intake, elevator, shoulder))
//
//                .strafeToConstantHeading(new Vector2d(36, -55))
//                //.splineToConstantHeading(new Vector2d(35, -61.4), Math.toRadians(-90))
//                .waitSeconds(0.15)
//                .stopAndAdd(SpecimenPickSeq.AutoSecimenPickAction(intake, elevator, shoulder))
//
//                // .strafeToConstantHeading(new Vector2d(40,-56))
//
//                // .strafeToConstantHeading(new Vector2d(4,-31))++-
//
//                .afterTime(0.6, SpecimenDropSeq.AutoSpecimenPreDropAction(intake, elevator, shoulder))
//                .strafeToConstantHeading(new Vector2d(-1, -35.5))
//                // .setReversed(true)
//                // .splineToConstantHeading(new Vector2d(-1, -31), Math.toRadians(90))
//                .waitSeconds(0.2)
//                .stopAndAdd(SpecimenDropSeq.AutoSpecimenDropAction(intake, elevator, shoulder))
//                .waitSeconds(0.2)
//                .stopAndAdd(new InstantAction(() -> intake.GripperCommands(Intake.GripperState.FULL_OPEN)))
//
//
//                //TODO: ALLIANCE PICK
//
//                .afterTime(0.5, SpecimenPickSeq.AutoSecimenPrePickAction(intake, elevator, shoulder))
//
//                .strafeToConstantHeading(new Vector2d(36, -55))
//                // .splineToConstantHeading(new Vector2d(34.5, -61.4), Math.toRadians(-90))
//                .waitSeconds(0.15)
//                .stopAndAdd(SpecimenPickSeq.AutoSecimenPickAction(intake, elevator, shoulder))
//
//                // .strafeToConstantHeading(new Vector2d(40,-56))
//
//
//                .afterTime(0.6, SpecimenDropSeq.AutoSpecimenPreDropAction(intake, elevator, shoulder))
//                .strafeToConstantHeading(new Vector2d(-3, -34))
//                //.setReversed(true)
//                //.splineToConstantHeading(new Vector2d(7  , -32), Math.toRadians(90))
//                .waitSeconds(0.1)
//                .stopAndAdd(SpecimenDropSeq.AutoSpecimenDropAction(intake, elevator, shoulder))
//                .waitSeconds(0.2)
//                .stopAndAdd(() -> intake.GripperCommands(Intake.GripperState.FULL_OPEN))
//
//
//                //TODO: PARKING
//
////                .setReversed(true)
//                .afterTime(0.5, SpecimenDropSeq.AutoLastSpecimenAfterDropAction(intake, elevator, shoulder))
//
//                .splineToConstantHeading(new Vector2d(50, -56), Math.toRadians(0))
//
//                .waitSeconds(0.7)
//                .waitSeconds(50)
//
//                .waitSeconds(1)
//
//                .build();
////
////        // Todo ====================================  PID THREAD ======================================================================
////        pidThread = new Thread(() -> {
////            while (!Thread.currentThread().isInterrupted()) {
////                try {
////                    if (opModeIsActive()) {
////                        Resource.PIDState();
////                    } else {
////                        elevator.ElevatorCommand(Elevator.ElevatorState.INIT);
////                    }
////                    Thread.sleep(10);
////                } catch (InterruptedException e) {
////                    Thread.currentThread().interrupt();
////                }
////            }
////        });
////
//        // Todo ==================================== SHOULDER THREAD ======================================================================
////        shoulderThread = new Thread(() -> {
////            while (!Thread.currentThread().isInterrupted()) {
////                try {
////                    if (opModeIsActive()) {
////                        shoulder.updateMotionProfile();
////                    }
////                    Thread.sleep(10);
////                } catch (InterruptedException e) {
////                    Thread.currentThread().interrupt();
////                }
////            }
////        });
//
//        // Todo ====================================  MAIN CODE ======================================================================
//        if (opModeInInit()) {
//            Actions.runBlocking(SpecimenPickSeq.AutoPreloadSpecimenPickAction(intake, elevator, shoulder));
//        }
////
////        pidThread.start();
////        shoulderThread.start();
////        PIDFalse();
////        while (opModeIsActive()){
////            drive.updatePoseEstimate();
////        }
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            Actions.runBlocking(autoSequence);
//
//        }
//
////        pidThread.interrupt();
////        shoulderThread.interrupt();
//
//    }
//}
//
//
