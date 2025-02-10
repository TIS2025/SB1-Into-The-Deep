package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.globals;

public class elevator_ss {

    private RobotHardware robot;

    public Elevator elevator = Elevator.HOME;

    public enum Elevator {
        HOME,
        SAMPLEPREPICK,
        SAMPLEPICK,
        SAMPLEDROP,
        SAMPLEDROPPRELOAD,
        SPECIMENDROPPOSE,
        SPECIMENDROPPOSE1,
        SPECIMENDROP
    }
    public elevator_ss(RobotHardware robot) {
        this.robot = robot;
    }

    public void updateState(Elevator elevator) {
        this.elevator = elevator;
        switch (elevator){
            case HOME:
                extendTo(globals.ElevatorHome,1);
                break;
            case SAMPLEPREPICK:
                extendTo(globals.ElevatorSamplePrePick,1);
                break;
                case SAMPLEPICK:
                extendTo(globals.ElevatorSamplePick,1);
                break;
            case SAMPLEDROP:
                extendTo(globals.ElevatorSampleDrop,1);
                break;
            case SAMPLEDROPPRELOAD:
                extendTo(globals.ElevatorSampleDrop,1);
                break;
            case SPECIMENDROPPOSE:
                extendTo(globals.ElevatorSpecimenDropPose,1);
                break;
            case SPECIMENDROPPOSE1:
                extendTo(globals.ElevatorSpecimenDropPose1,1);
                break;
            case SPECIMENDROP:
                extendTo(globals.ElevatorSpecimenDrop,1);
        }
    }

    public void extendTo(int targetPos, double pow) {
        robot.lifterR.setTargetPosition(targetPos);
        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterR.setTargetPositionTolerance(10);
        robot.lifterR.setPower(pow);

        robot.lifterL.setTargetPosition(targetPos);
        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterL.setTargetPositionTolerance(10);
        robot.lifterL.setPower(pow);
    }

    public Action elevatorCommand(Elevator state) {
        return new InstantAction(() -> updateState(state));
    }

    public void resetElevator() {
        robot.lifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
