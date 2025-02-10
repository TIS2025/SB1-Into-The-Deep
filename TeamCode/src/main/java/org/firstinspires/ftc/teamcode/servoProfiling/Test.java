package org.firstinspires.ftc.teamcode.servoProfiling;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@Config
@TeleOp
public class Test extends LinearOpMode {
    RobotHardware robot=RobotHardware.getInstance();
    public  double totalAngle=0;
    public double prevAngle=0;
    public  static  double target1=0.7961;
    public static double target2=0.2361;
    public static double speed=0.05;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.a){
                runServo(target1,speed);
            } else if (gamepad1.b) {
                runServo(target2,speed);

            }


            telemetry.addData("prev",prevAngle);
            telemetry.addData("total",totalAngle);
            telemetry.update();
        }
    }
    public void runServo(double pow, double speed){
        totalAngle = pow*speed + prevAngle*(1 - speed);
        robot.intk_lshoulder.setPosition(totalAngle);
        robot.intk_rshoulder.setPosition(totalAngle);
        prevAngle = totalAngle;
    }
}
