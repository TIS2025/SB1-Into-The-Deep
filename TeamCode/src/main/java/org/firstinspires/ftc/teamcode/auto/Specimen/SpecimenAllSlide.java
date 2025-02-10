package org.firstinspires.ftc.teamcode.auto.Specimen;


import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.AUTOINIT;
import org.firstinspires.ftc.teamcode.Sequences.INIT;
import org.firstinspires.ftc.teamcode.Sequences.specimenDrop;
import org.firstinspires.ftc.teamcode.Sequences.specimenPick;
import org.firstinspires.ftc.teamcode.Sequences.specimenPreDrop;
import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

import java.util.Arrays;


@Autonomous(name = "Specimen 80(1+3 Fast)")
public class SpecimenAllSlide extends LinearOpMode{

    private RobotHardware robot = RobotHardware.getInstance();

    private MecanumDrive drive = null;

    private org.firstinspires.ftc.teamcode.Subsystem.intake_ss intake_ss;
    private outake_ss outake_ss;
    private elevator_ss elevator_ss;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        intake_ss = new intake_ss(robot);
        outake_ss = new outake_ss(robot);
        elevator_ss = new elevator_ss(robot);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Pose2d intialPose = new Pose2d(15, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);

        // Todo ==================================== VELO CONST ======================================================================
        VelConstraint baseConst45 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(45)));
        VelConstraint baseConst70 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(70)));
        VelConstraint baseConst80 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));
        VelConstraint baseConst90 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(90)));

        // Todo ==================================== ACC CONST ======================================================================

        AccelConstraint baseAccel50 = new ProfileAccelConstraint(-45.0, 80.0);

        TrajectoryActionBuilder Action1 = drive.actionBuilder(intialPose)

                // TODO - PRELOAD
                .afterTime(0.001,()-> new specimenPreDrop(intake_ss,outake_ss, elevator_ss, 1))
                .splineToConstantHeading(new Vector2d(9.5, -28), Math.toRadians(88),baseConst80)
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))
//                .splineToConstantHeading(new Vector2d(9.5, -40), Math.toRadians(90))
                .setReversed(true)

                // TODO - Sliding 1st Sample
                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
                .splineToLinearHeading(new Pose2d(new Vector2d(30, -40), Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(new Vector2d(46, -12), Math.toRadians(90)), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(47.5, -45), baseConst90, baseAccel50)


                // TODO - Sliding 2st Sample
                .splineToLinearHeading(new Pose2d(new Vector2d(55, -10), Math.toRadians(90)), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(58.5, -45), baseConst90, baseAccel50)

                //TODO - Sliding 3rd Sample
                .splineToLinearHeading(new Pose2d(new Vector2d(63, -10), Math.toRadians(90)), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(61, -45), baseConst90, baseAccel50)


                // TODO - Pick up 1st Specimen
                .splineToLinearHeading(new Pose2d(32.5,-62,Math.toRadians(90)),Math.toRadians(-90))
                .stopAndAdd(()->new specimenPreDrop(intake_ss , outake_ss , elevator_ss,2 ))

                // TODO - Droping 1st Specimen
                .afterTime(0.1,()->new specimenPreDrop(intake_ss,outake_ss,elevator_ss,1))
                .splineToLinearHeading(new Pose2d(6,-28.5, Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))

//                //TODO -  Pick up 2nd Specimen
                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.5,-62, Math.toRadians(90)),Math.toRadians(-85))
                .stopAndAdd(()->new specimenPreDrop(intake_ss , outake_ss , elevator_ss,2 ))
//
//                // TODO - Droping 2nd Specimen
                .afterTime(0.1,()->new specimenPreDrop(intake_ss,outake_ss,elevator_ss,1))
                .splineToLinearHeading(new Pose2d(3,-28.5, Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))

                //TODO -  Pick up 3rd Specimen
                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35.5,-62, Math.toRadians(90)),Math.toRadians(-85))
                .stopAndAdd(()->new specimenPreDrop(intake_ss , outake_ss , elevator_ss,2 ))

                // TODO - Droping 3rd Specimen
                .afterTime(0.1,()->new specimenPreDrop(intake_ss,outake_ss,elevator_ss,1))
                .strafeTo(new Vector2d(-0,-28))
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))


                //TODO - Parking
                .afterTime(0.2,()->new INIT(intake_ss,outake_ss,elevator_ss,'c'))
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(36,-58, Math.toRadians(90)),Math.toRadians(-45));
                .strafeTo(new Vector2d(36,-58));


        if (opModeInInit()) {
            new AUTOINIT(intake_ss, outake_ss, elevator_ss);
        }

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        Action1.build()
                )
        );
    }
}
