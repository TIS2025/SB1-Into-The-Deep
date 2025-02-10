package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class sampleIntake {
    public static final int a = 0;
    public sampleIntake(intake_ss intake, outake_ss outake, elevator_ss elevator )
    {
        Actions.runBlocking(
                new SequentialAction(
                        intake.iextendCommand(intake_ss.EXTENDER.INIT),
                        outake.oarmCommand(outake_ss.Arm.SAMPLEPICK),
                        outake.oelbowCommand(outake_ss.Elbow.SAMPLEPICK),
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
                        intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                        intake.iRollerCommand(intake_ss.ROLLER.INTAKE),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPREPICK)
                        )
                );
    }

    public sampleIntake(intake_ss intake, outake_ss outake, elevator_ss elevator,int a )
    {

            Actions.runBlocking(
                    new SequentialAction(
                            intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                            intake.iRollerCommand(intake_ss.ROLLER.INTAKE),
                            outake.oarmCommand(outake_ss.Arm.SAMPLEPICK),
                            outake.oelbowCommand(outake_ss.Elbow.SAMPLEPICK),
                            outake.ogripperCommand(outake_ss.Gripper.OPEN),
                            elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPREPICK)
                    )
            );
    }

    public sampleIntake(intake_ss intake, outake_ss outake, elevator_ss elevator,String Prelaod2 )
    {
        Actions.runBlocking(
                new SequentialAction(
                        intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                        intake.iRollerCommand(intake_ss.ROLLER.INTAKE),
                        outake.oarmCommand(outake_ss.Arm.SAMPLEPICK),
                        outake.oelbowCommand(outake_ss.Elbow.SAMPLEPICK),
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPREPICK),
                        intake.iextendCommand(intake_ss.EXTENDER.AUTOSAMPLE3)
                )
        );
    }

    public sampleIntake(intake_ss intake,elevator_ss elevator)
    {
        Actions.runBlocking(
                new SequentialAction(
                        intake.iSholderCommand(intake_ss.SHOLDER.PICK),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEDROP),
                        intake.iRollerCommand(intake_ss.ROLLER.INTAKE),
                        intake.iextendCommand(intake_ss.EXTENDER.AUTOSAMPLE2)
                )
        );
    }



}
