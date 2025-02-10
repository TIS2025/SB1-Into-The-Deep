package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

public class specimenDrop {
    public specimenDrop(outake_ss outake, elevator_ss elevator){
        Actions.runBlocking(
                new SequentialAction(
                        elevator.elevatorCommand(elevator_ss.Elevator.SPECIMENDROP),
                        new SleepAction(0.5),
                        outake.ogripperCommand(outake_ss.Gripper.OPEN)
                        )
        );
    }
}
