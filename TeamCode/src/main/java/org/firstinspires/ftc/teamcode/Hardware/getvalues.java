package org.firstinspires.ftc.teamcode.Hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "getvalues")
@Config
public class getvalues extends LinearOpMode
{
    private static final RobotHardware robot = RobotHardware.getInstance();


    public static double shoulder=globals.sholderInit;
    public static double angle= 0.5;
    public static double arm= 0.6;
    public static double elbow= globals.elbowINIT;

    public static double swipe= 0.8;


    public static double wrist= 0.5;
    public static double gripper= 0.2;
    public static int lifterPos= 0;



    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap,telemetry);
//        robot.lifterR.setDirection(DcMotorSimple.Direction.REVERSE);


        robot.lifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(opModeInInit())
        {
            robot.elbow.setPosition(globals.elbowINIT);
            robot.gripper.setPosition(globals.gripperOpen);
            robot.swipe.setPosition(0.8);
            servoPose(0.6);
            shoulder(globals.sholderInit);

            robot.lifterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lifterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForStart();
        while (opModeIsActive())
        {


            if(gamepad1.dpad_up)
            {
                shoulder+=0.001;
                shoulder(shoulder);
            }
            else if(gamepad1.dpad_down)
            {
                shoulder-=0.001;
                shoulder(shoulder);
            }

            if(gamepad2.dpad_up)
            {
                swipe+=0.001;
                robot.swipe.setPosition(swipe);
            }
            else if(gamepad2.dpad_down)
            {
                swipe-=0.001;
                robot.swipe.setPosition(swipe);
            }


//             if(gamepad1.left_bumper){
//                 robot.intake.setPower(0.5);
//             }
//
//             else if(gamepad1.right_bumper){
//                 robot.intake.setPower(0);
//             }





//             //-------------ELEVATOR---------------------
//
            if(gamepad1.dpad_right)
            {
                arm+=0.0005;
                servoPose(arm);
            }
            else if(gamepad1.dpad_left)
            {
                arm-=0.0005;
                servoPose(arm);
            }
            if(gamepad1.left_trigger>0)
            {
                elbow+=0.001;
                robot.elbow.setPosition(elbow);
            }
            else if(gamepad1.right_trigger>0)
            {
                elbow-=0.001;
                robot.elbow.setPosition(elbow);
            }


            if(gamepad2.a)
            {
                gripper+=0.001;
                robot.gripper.setPosition(gripper);
            }
            else if(gamepad2.b)
            {
                gripper-=0.001;
                robot.gripper.setPosition(gripper);
            }


            //

//             if(gamepad2.left_trigger>0){
//                 inc(0.6);
//             }
//             else if(gamepad2.right_trigger>0){
//                dec(0.6);
//             }


            telemetry.addData("Shoulder L : ",robot.intk_lshoulder.getPosition());
            telemetry.addData("Shoulder R : ",robot.intk_rshoulder.getPosition());

            telemetry.addData("arm: ",robot.arm.getPosition());
            telemetry.addData("Elbow: ",robot.elbow.getPosition());
            telemetry.addData("Gripper: ",robot.gripper.getPosition());
            telemetry.addData("Swipe: ",robot.swipe.getPosition());

            telemetry.addData("lifter 1: ",robot.lifterR.getCurrentPosition());
            telemetry.addData("lifter 2: ",robot.lifterL.getCurrentPosition());

            telemetry.addData("lifter 1 Current: ",robot.lifterR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lifter 2 Current: ",robot.lifterL.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
    }

    public static void shoulder(double pos)
    {
        robot.intk_lshoulder.setPosition(pos);
        robot.intk_rshoulder.setPosition(pos);
    }
    public void servoPose(double pose){
        robot.arm.setPosition(pose);
        robot.armL.setPosition(1 - pose);

    }
//
//
//
//    public static void setPosition(int pos,double pow)
//    {
//        robot.lifterR.setTargetPosition(pos);
//        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifterR.setPower(pow);
//
//        robot.lifterL.setTargetPosition(pos);
//        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifterL.setPower(pow);
//    }

//    public static void inc(double pow)
//    {
//        robot.lifterR.setTargetPosition(robot.lifterR.getCurrentPosition()-20);
//        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifterR.setPower(pow);
//
//        robot.lifterL.setTargetPosition(robot.lifterL.getCurrentPosition()-20);
//        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifterL.setPower(pow);
//    }


//    public static void dec(double pow)
//    {
//        robot.lifterR.setTargetPosition(robot.lifterR.getCurrentPosition()+20);
//        robot.lifterR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifterR.setPower(pow);
//
//        robot.lifterL.setTargetPosition(robot.lifterL.getCurrentPosition()+20);
//        robot.lifterL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifterL.setPower(pow);
//    }


}
