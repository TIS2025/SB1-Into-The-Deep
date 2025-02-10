package org.firstinspires.ftc.teamcode.auto.Specimen;


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


@Autonomous(name = "Specimen 80(2+2) Mechanism")
public class SpecimenSwipe80 extends LinearOpMode{
    VelConstraint speedup = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(90)));
    VelConstraint speedup80 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(80)));

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

//        new AutoInitSeq(intake,outtake,hanger);



        TrajectoryActionBuilder Action1 = drive.actionBuilder(intialPose)

                // TODO - PRELOAD
                .afterTime(0.001,()-> new specimenPreDrop(intake_ss,outake_ss, elevator_ss, 1))
                .splineToConstantHeading(new Vector2d(9.5, -26), Math.toRadians(88),speedup80)
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))
//                .splineToConstantHeading(new Vector2d(9.5, -40), Math.toRadians(90))
                .setReversed(true)

                // TODO - Sliding 1st Sample
                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
                .splineToLinearHeading(new Pose2d(36, -46, Math.toRadians(48)), Math.toRadians(35)) //34.5, -45.5
                .strafeToLinearHeading(new Vector2d(34, -43.0001), Math.toRadians((-48)))

//                .strafeToLinearHeading(new Vector2d(27,-40), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(47.5,-14 , Math.toRadians(90)),Math.toRadians(-40),speedup) // x - 42
//                .strafeToLinearHeading(new Vector2d(48,-48), Math.toRadians(90),speedup) // y -

                // TODO - Sliding 2st Sample
                .strafeToLinearHeading(new Vector2d(41.5, -43), Math.toRadians(48))//42.5, -43
                .strafeToLinearHeading(new Vector2d(40.00, -46), Math.toRadians((-48)))//new Vector2d(40.00, -44), Math.toRadians((-48))

//                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
//                .strafeToLinearHeading(new Vector2d(46.5,-40), Math.toRadians(90),speedup)
//                .splineToLinearHeading(new Pose2d(56,-14, Math.toRadians(90)),Math.toRadians(-40),speedup)
//                .strafeToLinearHeading(new Vector2d(56,-37.5), Math.toRadians(90),speedup)

                // TODO - Sliding 3rd Sample
                .strafeToLinearHeading(new Vector2d(52, -44), Math.toRadians(48))//45
                .strafeToLinearHeading(new Vector2d(52.00, -46), Math.toRadians((-48)))//new Vector2d(40.00, -44), Math.toRadians((-48))


                // TODO - Pick up 1st Specimen
                .splineToLinearHeading(new Pose2d(32.5,-62,Math.toRadians(90)),Math.toRadians(-90))
                .stopAndAdd(()->new specimenPreDrop(intake_ss , outake_ss , elevator_ss,2 ))

                // TODO - Droping 1st Specimen
                .afterTime(0.1,()->new specimenPreDrop(intake_ss,outake_ss,elevator_ss,1))
                .splineToLinearHeading(new Pose2d(2,-28.5, Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))

//                //TODO -  Pick up 2nd Specimen
                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36.5,-62, Math.toRadians(90)),Math.toRadians(-45))
                .stopAndAdd(()->new specimenPreDrop(intake_ss , outake_ss , elevator_ss,2 ))
//
//                // TODO - Droping 2nd Specimen
                .afterTime(0.1,()->new specimenPreDrop(intake_ss,outake_ss,elevator_ss,1))
                .splineToLinearHeading(new Pose2d(0.5,-28.5, Math.toRadians(90)),Math.toRadians(90))
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))

                //TODO -  Pick up 3rd Specimen
                .afterTime(0.1,()->new specimenPick(intake_ss,outake_ss,elevator_ss))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(35.5,-62, Math.toRadians(90)),Math.toRadians(-45))
                .stopAndAdd(()->new specimenPreDrop(intake_ss , outake_ss , elevator_ss,2 ))

                // TODO - Droping 3rd Specimen
                .afterTime(0.1,()->new specimenPreDrop(intake_ss,outake_ss,elevator_ss,1))
                .strafeTo(new Vector2d(-1,-28))
                .stopAndAdd(() -> new specimenDrop(outake_ss, elevator_ss))


                //TODO - Parking
                .afterTime(0.2,()->new INIT(intake_ss,outake_ss,elevator_ss,'c'))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36,-58, Math.toRadians(90)),Math.toRadians(-45));

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
