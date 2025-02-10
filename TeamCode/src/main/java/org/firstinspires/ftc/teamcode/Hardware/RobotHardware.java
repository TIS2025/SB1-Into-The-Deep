package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware
{

    //-------------INTAKE-------------
    public Servo intk_lshoulder;
    public Servo intk_rshoulder;
    public Servo swipe;

    public DcMotorEx extender;

    public CRServoImplEx rollerL;
    public CRServoImplEx rollerR;

    //-------------LIFTER-------------

    public Servo arm;
    public Servo armL;
    public Servo elbow;
    public Servo gripper;
    public DcMotorEx lifterR;
    public DcMotorEx lifterL;

    public DcMotorEx HangerMotor = null;

    // Color Sensor
    public RevColorSensorV3 colorSensor;
    public DigitalChannel beamBreaker=null;


    public static RobotHardware instance = null;

    public boolean enabled;
    private HardwareMap hardwareMap;


    // TODO HARDWAREMAP
    public static RobotHardware getInstance()
    {
        if(instance == null)
        {
            instance= new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;

        //---------------------------LIFTER------------------------------

        lifterR =hardwareMap.get(DcMotorEx.class,"lifterR");
        lifterL =hardwareMap.get(DcMotorEx.class,"lifterL");

        lifterR.setDirection(DcMotorSimple.Direction.REVERSE);

//        lifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //---------------------------OUTAKE------------------------------
        arm= hardwareMap.get(Servo.class,"arm");
        armL= hardwareMap.get(Servo.class,"armL");
        gripper=hardwareMap.get(Servo.class,"gripper");
        elbow=hardwareMap.get(Servo.class,"elbow");


        //---------------------------INTAKE------------------------------

        intk_lshoulder=hardwareMap.get(Servo.class,"ils");
        intk_rshoulder=hardwareMap.get(Servo.class,"irs");
        swipe = hardwareMap.get(Servo.class,"swipe");

//        intake=hardwareMap.get(DcMotorEx.class,"intake");
        rollerL=hardwareMap.get(CRServoImplEx.class,"rollerL");
        rollerR=hardwareMap.get(CRServoImplEx.class,"rollerR");

        extender=hardwareMap.get(DcMotorEx.class,"extender");

        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        // Hanger
        HangerMotor = hardwareMap.get(DcMotorEx.class, "Hanger");
        HangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
