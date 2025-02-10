package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class specimenPick {

    public specimenPick(intake_ss inttake, outake_ss outake, elevator_ss elevator){

        Actions.runBlocking(
                new ParallelAction(
                        inttake.iSholderCommand(intake_ss.SHOLDER.DISCARD),
                        inttake.iextendCommand(intake_ss.EXTENDER.INIT),
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
                        elevator.elevatorCommand(elevator_ss.Elevator.HOME),
                        outake.oarmCommand(outake_ss.Arm.SPECIMENPICK),
                        outake.oelbowCommand(outake_ss.Elbow.SPECIMENPICK),
                        new SleepAction(0.2),
                        inttake.iSholderCommand(intake_ss.SHOLDER.TRANSFER)
                        )
        );
    }

}
