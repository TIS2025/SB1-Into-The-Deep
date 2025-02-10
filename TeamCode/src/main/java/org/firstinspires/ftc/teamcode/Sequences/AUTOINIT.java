package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class AUTOINIT {

    public AUTOINIT(intake_ss intake, outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        intake.iextendCommand(intake_ss.EXTENDER.INIT),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        outake.oarmCommand(outake_ss.Arm.AUTOINIT),
                        outake.oelbowCommand(outake_ss.Elbow.AUTOINIT),
                        elevator.elevatorCommand(elevator_ss.Elevator.HOME),
                        intake.iSholderCommand(intake_ss.SHOLDER.INIT),
                        new SleepAction(2),
                        outake.ogripperCommand(outake_ss.Gripper.CLOSE)
                        )
        );

    }

    public AUTOINIT(intake_ss intake, outake_ss outake, elevator_ss elevator,int a){
        Actions.runBlocking(
                new SequentialAction(
                        intake.iextendCommand(intake_ss.EXTENDER.INIT),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        outake.oarmCommand(outake_ss.Arm.AUTOINIT1),
                        outake.oelbowCommand(outake_ss.Elbow.INIT),
                        elevator.elevatorCommand(elevator_ss.Elevator.HOME),
                        intake.iSholderCommand(intake_ss.SHOLDER.INIT),
                        new SleepAction(3),
                        outake.ogripperCommand(outake_ss.Gripper.CLOSE)
                )
        );

    }

}
