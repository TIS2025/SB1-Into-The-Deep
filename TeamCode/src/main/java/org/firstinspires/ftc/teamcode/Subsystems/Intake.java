package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Intake {

    public RobotHardware robot;

    public Intake(RobotHardware robot){this.robot=robot;}

    //Servo states
    public enum IntakeServo1State {
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    public enum IntakeServo2State {
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    public enum IntakeServo3State {
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    public enum IntakeMotorState{
        ON,
        OFF,
        INIT
    }

    IntakeServo1State servo1State = IntakeServo1State.INIT;
    IntakeServo2State servo2State = IntakeServo2State.INIT;
    IntakeServo3State servo3State = IntakeServo3State.INIT;
    IntakeMotorState motorState = IntakeMotorState.INIT;

    public InstantAction SetServo1(IntakeServo1State state){
        switch (state){
            case INIT:
                servo1State = IntakeServo1State.INIT;
                return new InstantAction(()->SetServo1(ServoConst.intakeServo1Init));
            case STATE1:
                servo1State = IntakeServo1State.STATE1;
                return new InstantAction(()->SetServo1(ServoConst.intakeServo1State1));
            case STATE2:
                servo1State = IntakeServo1State.STATE2;
                return new InstantAction(()->SetServo1(ServoConst.intakeServo1State2));
            case STATE3:
                servo1State = IntakeServo1State.STATE3;
                return new InstantAction(()->SetServo1(ServoConst.intakeServo1State3));
            default:
                servo1State = IntakeServo1State.INIT;
                return new InstantAction(()->SetServo1(0.5));
        }
    }
    public InstantAction SetServo2(IntakeServo2State state){
        switch (state){
            case INIT:
                servo2State = IntakeServo2State.INIT;
                return new InstantAction(()->SetServo2(ServoConst.intakeServo2Init));
            case STATE1:
                servo2State = IntakeServo2State.STATE1;
                return new InstantAction(()->SetServo2(ServoConst.intakeServo2State1));
            case STATE2:
                servo2State = IntakeServo2State.STATE2;
                return new InstantAction(()->SetServo2(ServoConst.intakeServo2State2));
            case STATE3:
                servo2State = IntakeServo2State.STATE3;
                return new InstantAction(()->SetServo2(ServoConst.intakeServo2State3));
            default:
                servo2State = IntakeServo2State.INIT;
                return new InstantAction(()->SetServo2(0.5));
        }
    }
    public InstantAction SetServo3(IntakeServo3State state){
        switch (state){
            case INIT:
                servo3State = IntakeServo3State.INIT;
                return new InstantAction(()->SetServo3(ServoConst.intakeServo3Init));
            case STATE1:
                servo3State = IntakeServo3State.STATE1;
                return new InstantAction(()->SetServo3(ServoConst.intakeServo3State1));
            case STATE2:
                servo3State = IntakeServo3State.STATE2;
                return new InstantAction(()->SetServo3(ServoConst.intakeServo3State2));
            case STATE3:
                servo3State = IntakeServo3State.STATE3;
                return new InstantAction(()->SetServo3(ServoConst.intakeServo3State3));
            default:
                servo3State = IntakeServo3State.INIT;
                return new InstantAction(()->SetServo3(0.5));
        }
    }

    public InstantAction SetMotor(IntakeMotorState state){
        switch (state){
            case ON:
                motorState = IntakeMotorState.ON;
                return new InstantAction(()->robot.intakeMotor.setPower(1));
            case OFF:
                motorState = IntakeMotorState.OFF;
                return new InstantAction(()->robot.intakeMotor.setPower(0));
            default:
                motorState = IntakeMotorState.INIT;
                return new InstantAction(()->robot.intakeMotor.setPower(0));
        }
    }

    private void SetServo1(double position){robot.intake1.setPosition(position);}
    private void SetServo2(double position){robot.intake2.setPosition(position);}
    private void SetServo3(double position){robot.intake3.setPosition(position);}

    public InstantAction IncServo1by0_1(){
        return new InstantAction(()->SetServo1(robot.intake1.getPosition()+0.1));
    }
    public InstantAction IncServo1by0_01(){
        return new InstantAction(()->SetServo1(robot.intake1.getPosition()+0.01));
    }
    public InstantAction DecServo1by0_1(){
        return new InstantAction(()->SetServo1(robot.intake1.getPosition()-0.1));
    }
    public InstantAction DecServo1by0_01(){
        return new InstantAction(()->SetServo1(robot.intake1.getPosition()-0.01));
    }

    public InstantAction IncServo2by0_1(){
        return new InstantAction(()->SetServo2(robot.intake2.getPosition()+0.1));
    }
    public InstantAction IncServo2by0_01(){
        return new InstantAction(()->SetServo2(robot.intake2.getPosition()+0.01));
    }
    public InstantAction DecServo2by0_1(){
        return new InstantAction(()->SetServo2(robot.intake2.getPosition()-0.1));
    }
    public InstantAction DecServo2by0_01(){
        return new InstantAction(()->SetServo2(robot.intake2.getPosition()-0.01));
    }

    public InstantAction IncServo3by0_1(){
        return new InstantAction(()->SetServo3(robot.intake3.getPosition()+0.1));
    }
    public InstantAction IncServo3by0_01(){
        return new InstantAction(()->SetServo3(robot.intake3.getPosition()+0.01));
    }
    public InstantAction DecServo3by0_1(){
        return new InstantAction(()->SetServo3(robot.intake3.getPosition()-0.1));
    }
    public InstantAction DecServo3by0_01(){
        return new InstantAction(()->SetServo3(robot.intake3.getPosition()-0.01));
    }
}

