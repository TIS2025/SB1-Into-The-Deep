package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Sequences.InitSeq;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lifter;
import org.firstinspires.ftc.teamcode.Subsystems.Picker;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;

@TeleOp(name = "get values")
public class getValues extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);

        Lifter lifter = new Lifter(robot);
        Slider slider = new Slider(robot);
        Intake intake = new Intake(robot);
        Picker picker = new Picker(robot);

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        //Init robot
        new InitSeq(intake,slider,lifter,picker);

        waitForStart();
        while (opModeIsActive()){
            P1.copy(C1);
            C1.copy(gamepad1);

            //Drivetrain
            robot.drive.setDrivePowers(driveCommand(C1));





        }
    }

    public PoseVelocity2d driveCommand(Gamepad gamepad){
        double drive = -gamepad.left_stick_y * 0.5;
        double strafe = -gamepad.left_stick_x * 0.5;
        double turn = -gamepad.right_stick_x * 0.5;
        return new PoseVelocity2d(new Vector2d(drive, strafe), turn);
    }
}
