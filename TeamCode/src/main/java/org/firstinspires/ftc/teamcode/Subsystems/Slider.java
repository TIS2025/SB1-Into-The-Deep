package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Globals.MotorConst;
import org.firstinspires.ftc.teamcode.Globals.ServoConst;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

public class Slider {

    RobotHardware robot;

    public Slider(RobotHardware robot){this.robot=robot;}

    public enum SliderState{
        INIT,
        STATE1,
        STATE2,
        STATE3
    }

    SliderState sliderState = SliderState.INIT;

    public void setSliderPos(int pos){
        robot.intakeSliderMotor.setTargetPosition(pos);
        robot.intakeSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeSliderMotor.setPower(1);
    }

    public InstantAction SetSlider(SliderState state){
        switch (state){
            case INIT:
                sliderState = SliderState.INIT;
                return new InstantAction(()->setSliderPos(MotorConst.sliderInit));
            case STATE1:
                sliderState = SliderState.STATE1;
                return new InstantAction(()->setSliderPos(MotorConst.sliderState1));
            case STATE2:
                sliderState  = SliderState.STATE2;
                return new InstantAction(()->setSliderPos(MotorConst.sliderState2));
            case STATE3:
                sliderState = SliderState.STATE3;
                return new InstantAction(()->setSliderPos(MotorConst.sliderState3));
            default:
                sliderState = SliderState.INIT;
                return new InstantAction(()->setSliderPos(0));
        }
    }

    public boolean isBusy(){
        return robot.intakeSliderMotor.isBusy();
    }

    public void stopSlider(){
        robot.intakeSliderMotor.setPower(0);
    }

    public InstantAction IncSliderby100(){
        return new InstantAction(()->setSliderPos(robot.intakeSliderMotor.getCurrentPosition()+100));
    }
    public InstantAction DecSliderby100(){
        return new InstantAction(()->setSliderPos(robot.intakeSliderMotor.getCurrentPosition()-100));
    }
    public InstantAction IncSliderby5(){
        return new InstantAction(()->setSliderPos(robot.intakeSliderMotor.getCurrentPosition()+5));
    }
    public InstantAction DecSliderby5(){
        return new InstantAction(()->setSliderPos(robot.intakeSliderMotor.getCurrentPosition()-5));
    }
}
