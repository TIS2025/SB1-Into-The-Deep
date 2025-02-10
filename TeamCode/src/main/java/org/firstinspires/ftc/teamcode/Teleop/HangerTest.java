package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class
HangerTest extends LinearOpMode {

    public DcMotorEx HangerMotor = null;
    public static int increment = 0;
    public static int HangPosOn = 15370; // 5200;
    public static int HangPosOff = 10444; // 1600;
    @Override
    public void runOpMode() throws InterruptedException {
        HangerMotor = hardwareMap.get(DcMotorEx.class, "Hanger");
//        HangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        increment = 0;
        waitForStart();
        while (opModeIsActive()){
            HangerMotor.setTargetPosition(increment);
            HangerMotor.setPower(1);
            HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(gamepad1.a){
                increment = HangerMotor.getCurrentPosition() + 50;
            }
            if(gamepad1.b){
                increment = HangerMotor.getCurrentPosition() - 50;
            }
            if(gamepad1.x){
                increment = HangPosOn;
            }
            if(gamepad1.y){
                increment = HangPosOff;
            }


            telemetry.addData("Position: ", HangerMotor.getCurrentPosition());
            telemetry.addData("Position incremental: ", increment);
            telemetry.addData("Hanger Pos: ", increment);
            telemetry.addData("Current: ", HangerMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }
}