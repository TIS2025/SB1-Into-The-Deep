package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Picker {
    RobotHardware robot;

    public Picker(RobotHardware robot){this.robot = robot;}

    public enum PickerServo1State {
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    public enum PickerServo2State {
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    public enum PickerServo3State {
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    PickerServo1State servo1State = PickerServo1State.INIT;
    PickerServo2State servo2State = PickerServo2State.INIT;
    PickerServo3State servo3State = PickerServo3State.INIT;

    public InstantAction SetServo1(PickerServo1State state){
        switch (state){
            case INIT:
                servo1State = PickerServo1State.INIT;
                return new InstantAction(()->SetServo1(ServoConst.pickerServo1Init));
            case STATE1:
                servo1State = PickerServo1State.STATE1;
                return new InstantAction(()->SetServo1(ServoConst.pickerServo1State1));
            case STATE2:
                servo1State = PickerServo1State.STATE2;
                return new InstantAction(()->SetServo1(ServoConst.pickerServo1State2));
            case STATE3:
                servo1State = PickerServo1State.STATE3;
                return new InstantAction(()->SetServo1(ServoConst.pickerServo1State3));
            default:
                servo1State = PickerServo1State.INIT;
                return new InstantAction(()->SetServo1(0.5));
        }
    }
    public InstantAction SetServo2(PickerServo2State state){
        switch (state){
            case INIT:
                servo2State = PickerServo2State.INIT;
                return new InstantAction(()->SetServo2(ServoConst.pickerServo2Init));
            case STATE1:
                servo2State = PickerServo2State.STATE1;
                return new InstantAction(()->SetServo2(ServoConst.pickerServo2State1));
            case STATE2:
                servo2State = PickerServo2State.STATE2;
                return new InstantAction(()->SetServo2(ServoConst.pickerServo2State2));
            case STATE3:
                servo2State = PickerServo2State.STATE3;
                return new InstantAction(()->SetServo2(ServoConst.pickerServo2State3));
            default:
                servo2State = PickerServo2State.INIT;
                return new InstantAction(()->SetServo2(0.5));
        }
    }
    public InstantAction SetServo3(PickerServo3State state){
        switch (state){
            case INIT:
                servo3State = PickerServo3State.INIT;
                return new InstantAction(()->SetServo3(ServoConst.pickerServo3Init));
            case STATE1:
                servo3State = PickerServo3State.STATE1;
                return new InstantAction(()->SetServo3(ServoConst.pickerServo3State1));
            case STATE2:
                servo3State = PickerServo3State.STATE2;
                return new InstantAction(()->SetServo3(ServoConst.pickerServo3State2));
            case STATE3:
                servo3State = PickerServo3State.STATE3;
                return new InstantAction(()->SetServo3(ServoConst.pickerServo3State3));
            default:
                servo3State = PickerServo3State.INIT;
                return new InstantAction(()->SetServo3(0.5));
        }
    }

    private void SetServo1(double position){robot.intake1.setPosition(position);}
    private void SetServo2(double position){robot.intake2.setPosition(position);}
    private void SetServo3(double position){robot.intake3.setPosition(position);}

    public InstantAction IncServo1by0_1(){
        return new InstantAction(()->SetServo1(robot.pick_drop1.getPosition()+0.1));
    }
    public InstantAction IncServo1by0_01(){
        return new InstantAction(()->SetServo1(robot.pick_drop1.getPosition()+0.01));
    }
    public InstantAction DecServo1by0_1(){
        return new InstantAction(()->SetServo1(robot.pick_drop1.getPosition()-0.1));
    }
    public InstantAction DecServo1by0_01(){
        return new InstantAction(()->SetServo1(robot.pick_drop1.getPosition()-0.01));
    }

    public InstantAction IncServo2by0_1(){
        return new InstantAction(()->SetServo2(robot.pick_drop2.getPosition()+0.1));
    }
    public InstantAction IncServo2by0_01(){
        return new InstantAction(()->SetServo2(robot.pick_drop2.getPosition()+0.01));
    }
    public InstantAction DecServo2by0_1(){
        return new InstantAction(()->SetServo2(robot.pick_drop2.getPosition()-0.1));
    }
    public InstantAction DecServo2by0_01(){
        return new InstantAction(()->SetServo2(robot.pick_drop2.getPosition()-0.01));
    }

    public InstantAction IncServo3by0_1(){
        return new InstantAction(()->SetServo3(robot.pick_drop3.getPosition()+0.1));
    }
    public InstantAction IncServo3by0_01(){
        return new InstantAction(()->SetServo3(robot.pick_drop3.getPosition()+0.01));
    }
    public InstantAction DecServo3by0_1(){
        return new InstantAction(()->SetServo3(robot.pick_drop3.getPosition()-0.1));
    }
    public InstantAction DecServo3by0_01(){
        return new InstantAction(()->SetServo3(robot.pick_drop3.getPosition()-0.01));
    }

}
