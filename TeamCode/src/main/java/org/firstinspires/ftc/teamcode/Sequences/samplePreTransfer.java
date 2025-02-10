package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class samplePreTransfer {
    public samplePreTransfer(intake_ss intake){
        Actions.runBlocking(
                new SequentialAction(
                        intake.iextendCommand(intake_ss.EXTENDER.INIT),
                        intake.iRollerCommand(intake_ss.ROLLER.OFF),
                        intake.iSholderCommand(intake_ss.SHOLDER.TRANSFER)

                )
        );
    }

    public static Action samplePreTransfer(intake_ss intake, outake_ss outake, elevator_ss elevator){
            return new SequentialAction(
                    intake.iextendCommand(intake_ss.EXTENDER.INIT),
                    intake.iSholderCommand(intake_ss.SHOLDER.TRANSFER),
                    new SleepAction(0.4),
                    elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPICK),
                    new SleepAction(0.35),
                    intake.iRollerCommand(intake_ss.ROLLER.OFF),
                    outake.ogripperCommand(outake_ss.Gripper.CLOSE),
                    new SleepAction(0.2)
            );
    }

}
