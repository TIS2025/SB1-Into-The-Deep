package org.firstinspires.ftc.teamcode.Teleop;

import static com.qualcomm.robotcore.util.Range.clip;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.globals;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Sequences.INIT;
import org.firstinspires.ftc.teamcode.Sequences.sampleDrop;
import org.firstinspires.ftc.teamcode.Sequences.sampleDropPose;
import org.firstinspires.ftc.teamcode.Sequences.sampleIntake;
import org.firstinspires.ftc.teamcode.Sequences.samplePreTransfer;
import org.firstinspires.ftc.teamcode.Sequences.specimenDrop;
import org.firstinspires.ftc.teamcode.Sequences.specimenPick;
import org.firstinspires.ftc.teamcode.Sequences.specimenPreDrop;
import org.firstinspires.ftc.teamcode.Subsystem.elevator_ss;
import org.firstinspires.ftc.teamcode.Subsystem.intake_ss;
import org.firstinspires.ftc.teamcode.Subsystem.outake_ss;

import java.util.ArrayList;
import java.util.List;
@TeleOp

public class SB_BLUE extends LinearOpMode {

    private MecanumDrive mecanumDrive;

    private intake_ss intake_ss;
    private outake_ss outake_ss;
    private elevator_ss elevator_ss;

    private final RobotHardware robot = RobotHardware.getInstance();

    public static int ElevatorPos;
    public static int xExtenseionPos;

    public static List<Action> runningActions = new ArrayList<>();
    private boolean detectionFlag = false;
    private boolean colorFlag = false;
    private boolean gripFlag = false;
    private boolean dropFlag = false;
    private boolean teleon = false;


    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        robot.init(hardwareMap, telemetry);

        intake_ss = new intake_ss(robot);
        outake_ss = new outake_ss(robot);
        elevator_ss = new elevator_ss(robot);

        NormalizedRGBA rgba;
        float[] hsv;
        double distance;
        robot.colorSensor.setGain((float) globals.gain);

        if (opModeInInit()) {
            new INIT(intake_ss, outake_ss, elevator_ss);
            teleon = false ;
        }

        while (opModeInInit()) {
            gamepad1.rumble(1, 1, 400);
            gamepad2.rumble(1, 1, 400);

            if (gamepad1.dpad_up) {
                ElevatorPos += 5;
            } else if (gamepad1.dpad_down) {
                ElevatorPos -= 5;
            }

            if (gamepad1.left_bumper) {
                xExtenseionPos += 5;
            } else if (gamepad1.right_bumper) {
                xExtenseionPos -= 5;
            }

            if (gamepad1.start) {
                xExtenseionPos = 0;
                ElevatorPos = 0;
            }


            elevator_ss.extendTo(ElevatorPos, 1);
            intake_ss.extender(xExtenseionPos, 1);
        }
        elevator_ss.resetElevator();
        intake_ss.resetExtender();
//        new INIT(intake_ss, outake_ss, elevator_ss,'c');

        waitForStart();
        while (opModeIsActive()) {

            if(!teleon){
                Actions.runBlocking(
                        new SequentialAction(
                                intake_ss.iSholderCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.SHOLDER.TRANSFER)
                        )
                );

                teleon = true;
            }

            runningActions =  updateAction();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (gamepad1.back) {
                new INIT(intake_ss, outake_ss, elevator_ss,'c');
                colorFlag = false;
                detectionFlag = false;
                gripFlag = false;
                dropFlag = false;
            }


            //TODO Color detection --> Do Not Change
            rgba = robot.colorSensor.getNormalizedColors();
            distance = robot.colorSensor.getDistance(DistanceUnit.MM);
            hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue);

            // TODO HSV RED
            if (((hsv[0] < 26) && (hsv[0] > 18) && distance < 30)) {
                globals.intakeItem = 1;
            }

            //TODO HSV YELLOW
            else if (((hsv[0] < 80) && (hsv[0] > 55) && (hsv[2] > 0.95)  && distance < 30)) {
                globals.intakeItem = 2;
            }

            // TODO HSV BLUE
            else if (((hsv[0] < 235) && (hsv[0] > 210) && distance < 30)) {
                globals.intakeItem = 3;
            }
            else {
                globals.intakeItem = 0;
            }

            //TODO: Color Detection Logic
            if(detectionFlag ) {
                if (globals.intakeItem == 3 || globals.intakeItem == 2) {
                    Actions.runBlocking(intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.OFF));
                    detectionFlag = false;
                    colorFlag = true;
                }
                else if (globals.intakeItem ==1){
                    Actions.runBlocking(
                            new SequentialAction(
                                    intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.OFF),
                                    intake_ss.iSholderCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.SHOLDER.DISCARD),
                                    intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.THROW),
                                    new SleepAction(1.5),
                                    intake_ss.iSholderCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.SHOLDER.PICK),
                                    intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.INTAKE)
                            )
                    );
                }
            }

            //TODO ========================================== MECANUM DRIVE ===============================================================
            mecanumDrive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(Math.pow(clip(-gamepad1.left_stick_y,-1,1),3), Math.pow(clip(-gamepad1.left_stick_x,-1,1),3)), -gamepad1.right_stick_x)
            );
            mecanumDrive.updatePoseEstimate();

            //TODO ======================================== INTAKE AND OUT-TAKE FROM SUBMERSIBLE ==========================================
            if(gamepad1.right_bumper) {
                new sampleIntake(intake_ss,outake_ss,elevator_ss);
                detectionFlag = true;
            }

            if(colorFlag && !detectionFlag){
                new SleepAction(2);
                new samplePreTransfer(intake_ss);
                colorFlag = false;
                gripFlag = true;
            }



            //TODO ======================================== Extension ==========================================

            if (gamepad1.left_trigger > 0.3 ) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(()->extender(410,0.65))
                ));


            }

            if (gamepad1.right_trigger > 0.3 ) {
                Actions.runBlocking(new SequentialAction(
                        new InstantAction(()->extender(830,0.65))
                ));
            }

            if(gamepad1.left_stick_y >0.3 && gripFlag){
                runningActions.add(sampleDropPose.sampleDropPose(outake_ss,elevator_ss,"Gripper"));
                gripFlag = false;
            }

            if(gamepad1.left_bumper){
                new sampleDropPose(intake_ss,outake_ss,elevator_ss);
            }

            if(gamepad1.dpad_down){
                new sampleDrop(outake_ss);
                dropFlag = true ;
            }

            if(-gamepad1.left_stick_y > 0.5 && dropFlag){
                new sampleDrop(outake_ss,elevator_ss);
                dropFlag = false;
            }

            if(gamepad1.dpad_left){
                new specimenPick(intake_ss,outake_ss,elevator_ss);
            }

            if(gamepad1.dpad_right){
                new specimenPreDrop(intake_ss,outake_ss,elevator_ss);
            }

            if (gamepad1.b) {
                new specimenDrop(outake_ss,elevator_ss);
            }

            if(gamepad2.dpad_up){
                hanger(-1250);
            }

            if(gamepad2.dpad_down){
                hanger(-6200);
            }

            if(gamepad2.b){
                Actions.runBlocking(
                        new SequentialAction(
                                intake_ss.iswipe(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.Swipe.OPEN),
                                new SleepAction(1),
                                intake_ss.iswipe(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.Swipe.CLOSE)
                        )
                );
            }

            if(gamepad2.x){
                Actions.runBlocking(
                        new SequentialAction(
                                intake_ss.iRollerCommand(org.firstinspires.ftc.teamcode.Subsystem.intake_ss.ROLLER.THROW)
                        )

                );
            }

            intake_ss.updateSholder();

            telemetry.addData("Colour Detectio",globals.intakeItem);
            telemetry.addData("Detection Flag",colorFlag);
            telemetry.addData("Colour Flag",detectionFlag);

            telemetry.addData("Lifter 1 Pose ",robot.lifterR.getCurrentPosition());
            telemetry.addData("Lifter 2 Pose ",robot.lifterL.getCurrentPosition());
            telemetry.addData("Lifter 1 Current ",robot.lifterR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lifter 2 Current ",robot.lifterL.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Extender Pose ",robot.extender.getCurrentPosition());
            telemetry.addData("Extender Current ",robot.extender.getCurrent(CurrentUnit.AMPS));

            telemetry.addLine()
                    .addData("Hue", hsv[0])
                    .addData("Saturation", hsv[1])
                    .addData("Value", hsv[2]);
            telemetry.update();


        }
    }

    public float[] rgbToHsv(float rNorm, float gNorm, float bNorm) {
        float[] hsv = new float[3];

        float max = Math.max(rNorm, Math.max(gNorm, bNorm));
        float min = Math.min(rNorm, Math.min(gNorm, bNorm));
        float delta = max - min;
        // Value
        hsv[2] = max;

        // Saturation
        hsv[1] = max == 0 ? 0 : delta / max;

        // Hue
        if (delta == 0) {
            hsv[0] = 0;
        } else {
            if (max == rNorm) {
                hsv[0] = (60 * ((gNorm - bNorm) / delta) + 360) % 360;
            } else if (max == gNorm) {
                hsv[0] = (60 * ((bNorm - rNorm) / delta) + 120) % 360;
            } else if (max == bNorm) {
                hsv[0] = (60 * ((rNorm - gNorm) / delta) + 240) % 360;
            }
        }

        return hsv;
    }

    public void extender(int targetPos,double pow){
        robot.extender.setTargetPosition(targetPos);
        robot.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extender.setTargetPositionTolerance(10);
        robot.extender.setPower(pow);
    }

    public void hanger(int position){
        robot.HangerMotor.setTargetPosition(position);
        robot.HangerMotor.setPower(1);
        robot.HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static List<Action> updateAction(){
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        List<Action> RemovableActions = new ArrayList<>();

        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
//        runningActions.removeAll(RemovableActions);
        return newActions;
    }
}
