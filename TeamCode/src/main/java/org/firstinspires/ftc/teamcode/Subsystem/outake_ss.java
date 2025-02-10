package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.globals;

public class outake_ss {
    private RobotHardware robot;

    public Arm arm= Arm.INIT;
    public Elbow elbow= Elbow.INIT;
    public Gripper gripper= Gripper.OPEN;

    public enum Arm
    {
        INIT,
        AUTOINIT,
        AUTOINIT1,
        SAMPLEPICK,
        SAMPLEDROP,
        SPECIMENPICK,
        SPECIMENDROP,
        SPECIMENPRELOAD
    }

    public enum Elbow
    {
        INIT,
        AUTOINIT,
        SAMPLEPICK,
        SAMPLEDROP,
        SPECIMENPICK,
        SPECIMENDROP,
        SPECIMENPRELOAD,
    }

    public enum Gripper
    {
        OPEN,
        CLOSE,
    }

    public outake_ss(RobotHardware robot) {
        this.robot = robot;
    }


    public void updateState(Arm state){
        this.arm = state;
        switch (state){
            case INIT:
                servoPose(globals.armINIT);
                break;
            case AUTOINIT:
                servoPose(globals.armAUTOINIT);
                break;
            case AUTOINIT1:
                servoPose(globals.armAUTOINIT1);
                break;
            case SAMPLEPICK:
                servoPose(globals.armSampleTransfer);
                break;
            case SAMPLEDROP:
                servoPose(globals.armDrop);
                break;
            case SPECIMENPICK:
                servoPose(globals.armSpecimenPick);
                break;
            case SPECIMENDROP:
                servoPose(globals.armSpecimenDrop);
                break;
            case SPECIMENPRELOAD:
                servoPose(globals.armSpecimenDropPreload);
                break;
        }
    }
    public void updateState(Gripper state){
        this.gripper = state;
        switch (state){
            case OPEN:
                robot.gripper.setPosition(globals.gripperOpen);
                break;
            case CLOSE:
                robot.gripper.setPosition(globals.gripperClose);
                break;
        }
    }

    public void updateState(Elbow state){
        this.elbow = state;
        switch (state){
            case INIT:
                robot.elbow.setPosition(globals.elbowINIT);
                break;
                case AUTOINIT:
                robot.elbow.setPosition(globals.elbowAUTOINIT);
                break;
            case SAMPLEPICK:
                robot.elbow.setPosition(globals.elbowSampleTransfer);
                break;
            case SAMPLEDROP:
                robot.elbow.setPosition(globals.elbowDrop);
                break;
            case SPECIMENPICK:
                robot.elbow.setPosition(globals.elbowSpecimenPick);
                break;
            case SPECIMENDROP:
                robot.elbow.setPosition(globals.elbowSpecimenDrop);
                break;
            case SPECIMENPRELOAD:
                robot.elbow.setPosition(globals.elbowSpecimenDropPreload);
                break;
        }
    }

    public void servoPose(double pose){
        robot.arm.setPosition(pose);
        robot.armL.setPosition(1 - pose);

    }

    public Action oarmCommand(Arm state) {
        return new InstantAction(() -> updateState(state));
    }

    public Action oelbowCommand(Elbow state) {
        return new InstantAction(() -> updateState(state));
    }

    public Action ogripperCommand(Gripper state) {
        return new InstantAction(() -> updateState(state));
    }

}
