package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class INIT {

    public INIT(intake_ss intake, outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                        new SleepAction(1),
                        intake.iextendCommand(intake_ss.EXTENDER.INIT),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        outake.oarmCommand(outake_ss.Arm.INIT),
                        outake.oelbowCommand(outake_ss.Elbow.INIT),
                        outake.ogripperCommand(outake_ss.Gripper.CLOSE),
                        elevator.elevatorCommand(elevator_ss.Elevator.HOME),
                        intake.iSholderCommand(intake_ss.SHOLDER.INIT1)
                        )
        );

    }

    public INIT(intake_ss intake, outake_ss outake, elevator_ss elevator,String autoINIT )
    {
        Actions.runBlocking(
                new SequentialAction(
                        intake.iSholderCommand(intake_ss.SHOLDER.PICK1),
                        outake.oarmCommand(outake_ss.Arm.SAMPLEPICK),
                        outake.oelbowCommand(outake_ss.Elbow.SAMPLEPICK),
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
//                        intake.iSholderCommand(intake_ss.SHOLDER.INIT),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPREPICK)
                )
        );
    }

    public INIT(intake_ss intake, outake_ss outake, elevator_ss elevator,Character c){
        Actions.runBlocking(
                new SequentialAction(
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
                        new SleepAction(0.2),
                        intake.iSholderCommand(intake_ss.SHOLDER.DISCARD),
                        new SleepAction(1),
                        intake.iextendCommand(intake_ss.EXTENDER.INIT),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        outake.oarmCommand(outake_ss.Arm.INIT),
                        outake.oelbowCommand(outake_ss.Elbow.INIT),
                        elevator.elevatorCommand(elevator_ss.Elevator.HOME),
                        intake.iSholderCommand(intake_ss.SHOLDER.INIT1)
                        )
        );

    }

}
