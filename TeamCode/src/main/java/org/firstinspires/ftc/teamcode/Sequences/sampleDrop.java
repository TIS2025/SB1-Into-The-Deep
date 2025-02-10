package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class sampleDrop {
    public sampleDrop(intake_ss intake, outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
                        new SleepAction(0.2),
                        outake.oarmCommand(outake_ss.Arm.SAMPLEPICK),
                        outake.oelbowCommand(outake_ss.Elbow.SAMPLEPICK),
                        new SleepAction(0.3),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPREPICK)
                )
        );
    }


    public sampleDrop(outake_ss outake){
        Actions.runBlocking(
                new SequentialAction(
                        outake.ogripperCommand(outake_ss.Gripper.OPEN),
                        new SleepAction(0.15),
                        outake.oarmCommand(outake_ss.Arm.SAMPLEPICK)
                )
        );
    }

    public sampleDrop(outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        outake.oarmCommand(outake_ss.Arm.SAMPLEPICK),
                        outake.oelbowCommand(outake_ss.Elbow.SAMPLEPICK),
                        elevator.elevatorCommand(elevator_ss.Elevator.SAMPLEPREPICK)
                )
        );
    }
}