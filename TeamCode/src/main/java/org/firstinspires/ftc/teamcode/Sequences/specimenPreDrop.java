package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class specimenPreDrop {

    public specimenPreDrop(intake_ss intake, outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        outake.ogripperCommand(outake_ss.Gripper.CLOSE),
                        new SleepAction(0.25),
                        elevator.elevatorCommand(elevator_ss.Elevator.SPECIMENDROPPOSE),
                        outake.oarmCommand(outake_ss.Arm.SPECIMENDROP),
                        outake.oelbowCommand(outake_ss.Elbow.SPECIMENDROP),
                        intake.iSholderCommand(intake_ss.SHOLDER.TRANSFER)
                )
        );
    }

    public specimenPreDrop(intake_ss intake,outake_ss outake, elevator_ss elevator,int a){
        // Preload
        if(a ==1){
            Actions.runBlocking(
                    new SequentialAction(
                            elevator.elevatorCommand(elevator_ss.Elevator.SPECIMENDROPPOSE),
                            outake.oarmCommand(outake_ss.Arm.SPECIMENPRELOAD),
                            outake.oelbowCommand(outake_ss.Elbow.SPECIMENPRELOAD)
                    )
            );
        }

        if(a ==2){
            Actions.runBlocking(
                    new SequentialAction(
                            outake.ogripperCommand(outake_ss.Gripper.CLOSE),
                            new SleepAction(0.3),
                            elevator.elevatorCommand(elevator_ss.Elevator.SPECIMENDROPPOSE1)
                    )
            );
        }
    }

}
