package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RobotHardware {

    //Drivetrain - naming consistent with Mecanumdrive file
    public MecanumDrive drive;
    DcMotorEx leftFront,leftBack,rightFront,rightBack;

    //Todo Intake for samples and extension
    public DcMotorEx intakeSliderMotor,intakeMotor;
    public Servo intake1,intake2,intake3;

    //Todo Lifter
    public DcMotorEx lifterMotor;

    //Todo Picker, Dropper for specimens
    public Servo pick_drop1,pick_drop2,pick_drop3;

    //Todo Color Sensor, Camera
    public RevColorSensorV3 colorSenor;

    public RobotHardware(HardwareMap hardwareMap){

        //Drivetrain init
        this.drive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),0));

        //Intake init
        this.intakeMotor = hardwareMap.get(DcMotorEx.class,"intMotor");
        this.intakeSliderMotor = hardwareMap.get(DcMotorEx.class,"intSlMotor");
        this.intake1 = hardwareMap.get(Servo.class,"servo1");
        this.intake2 = hardwareMap.get(Servo.class,"servo2");
        this.intake3 = hardwareMap.get(Servo.class,"servo3");

        //Lifter init
        this.lifterMotor = hardwareMap.get(DcMotorEx.class,"liftMotor");

        //Drop,Pick mech init
        this.pick_drop1 = hardwareMap.get(Servo.class,"pick1");
        this.pick_drop2 = hardwareMap.get(Servo.class,"pick2");
        this.pick_drop3 = hardwareMap.get(Servo.class,"pick3");

        //Color sensor init
        this.colorSenor = hardwareMap.get(RevColorSensorV3.class,"color");
    }
}
