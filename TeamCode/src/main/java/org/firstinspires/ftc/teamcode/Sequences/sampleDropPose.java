package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class sampleDropPose {
    public sampleDropPose(intake_ss intake, outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        intake.iRollerCommand(intake_ss.ROLLER.TRANSFER),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEDROP),
                        outake.oarmCommand(outake_ss.Arm.SAMPLEDROP),
                        outake.oelbowCommand(outake_ss.Elbow.SAMPLEDROP),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        intake.iSholderCommand(intake_ss.SHOLDER.INIT1)
                        )
        );
    }

    public sampleDropPose(intake_ss intake,outake_ss outake, elevator_ss elevator,int a) {
        //TODO - Preload Droping
        if(a == 1) {
            Actions.runBlocking(
                    new SequentialAction(
                            intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                            elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEDROPPRELOAD),
                            outake.oarmCommand(outake_ss.Arm.SAMPLEDROP),
                            outake.oelbowCommand(outake_ss.Elbow.SAMPLEDROP)
                    )
            );
        }

        else if(a == 2){
            Actions.runBlocking(
                    new SequentialAction(
                            intake.iRollerCommand(intake_ss.ROLLER.TRANSFER),
                            elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEDROP),
                            outake.oarmCommand(outake_ss.Arm.SAMPLEDROP),
                            outake.oelbowCommand(outake_ss.Elbow.SAMPLEDROP),
                            intake.iRollerCommand(intake_ss.ROLLER.OFF),
                            intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                            intake.iextendCommand(intake_ss.EXTENDER.AUTOSAMPLE)
                    )
            );
        }
        //TODO Submersible Sample Picking Sequence
        else if (a == 3){
            Actions.runBlocking(
                    new SequentialAction(
                            intake.iRollerCommand(intake_ss.ROLLER.TRANSFER),
                            elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEDROP),
                            outake.oarmCommand(outake_ss.Arm.SAMPLEDROP),
                            outake.oelbowCommand(outake_ss.Elbow.SAMPLEDROP),
                            intake.iRollerCommand(intake_ss.ROLLER.OFF),
                            intake.iSholderCommand(intake_ss.SHOLDER.INIT1)
                    )
            );
        }
    }

    public static Action sampleDropPose(outake_ss outake, elevator_ss elevator, String grip){
                 return new SequentialAction(
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPICK),
                        new SleepAction(0.2),
                        outake.ogripperCommand(outake_ss.Gripper.CLOSE)
        );
    }

}
