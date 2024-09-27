package org.firstinspires.ftc.teamcode.Sequences;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lifter;
import org.firstinspires.ftc.teamcode.Subsystems.Picker;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

public class InitSeq {
    public InitSeq(Intake intake, Slider slider, Lifter lifter, Picker picker){
        Actions.runBlocking(
                new SequentialAction(
                        intake.SetServo1(Intake.IntakeServo1State.INIT),
                        intake.SetServo2(Intake.IntakeServo2State.INIT),
                        intake.SetServo3(Intake.IntakeServo3State.INIT),
                        intake.SetMotor(Intake.IntakeMotorState.INIT),
                        slider.SetSlider(Slider.SliderState.INIT),
                        lifter.SetLifter(Lifter.LifterState.INIT),
                        picker.SetServo1(Picker.PickerServo1State.INIT),
                        picker.SetServo2(Picker.PickerServo2State.INIT),
                        picker.SetServo3(Picker.PickerServo3State.INIT)
                )
        );
    }
}
