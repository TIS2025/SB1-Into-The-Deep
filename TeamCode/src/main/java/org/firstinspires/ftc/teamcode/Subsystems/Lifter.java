package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Lifter {
    
    RobotHardware robot;

    public Lifter(RobotHardware robot){ this.robot = robot;}

    public enum LifterState{
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    LifterState lifterState = LifterState.INIT;

    public void setLifterPos(int pos){
        robot.lifterMotor.setTargetPosition(pos);
        robot.lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifterMotor.setPower(1);
    }

    public InstantAction SetLifter(LifterState state){
        switch (state){
            case INIT:
                lifterState = LifterState.INIT;
                return new InstantAction(()->setLifterPos(MotorConst.lifterInit));
            case STATE1:
                lifterState = LifterState.STATE1;
                return new InstantAction(()->setLifterPos(MotorConst.lifterState1));
            case STATE2:
                lifterState  = LifterState.STATE2;
                return new InstantAction(()->setLifterPos(MotorConst.lifterState2));
            case STATE3:
                lifterState = LifterState.STATE3;
                return new InstantAction(()->setLifterPos(MotorConst.lifterState3));
            default:
                lifterState = LifterState.INIT;
                return new InstantAction(()->setLifterPos(0));
        }
    }

    public boolean isBusy(){
        return robot.lifterMotor.isBusy();
    }

    public void stopLifter(){
        robot.lifterMotor.setPower(0);
    }

    public InstantAction IncLifterby100(){
        return new InstantAction(()->setLifterPos(robot.lifterMotor.getCurrentPosition()+100));
    }
    public InstantAction DecLifterby100(){
        return new InstantAction(()->setLifterPos(robot.lifterMotor.getCurrentPosition()-100));
    }
    public InstantAction IncLifterby5(){
        return new InstantAction(()->setLifterPos(robot.lifterMotor.getCurrentPosition()+5));
    }
    public InstantAction DecLifterby5(){
        return new InstantAction(()->setLifterPos(robot.lifterMotor.getCurrentPosition()-5));
    }
}

