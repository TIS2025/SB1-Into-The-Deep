package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.globals;

@Config
public class intake_ss {

    public SHOLDER sholder = SHOLDER.INIT;
    public ROLLER roller = ROLLER.OFF;
    public EXTENDER extender = EXTENDER.INIT;
    public Swipe swipe = Swipe.CLOSE;

    private RobotHardware robot;

    // Servo Profiling
    // Target and current positions and velocity
    public double armTargetPosition = globals.sholderSamplePick;   //TODO INIT Target position
    public double currentPosition = globals.sholderSamplePick; //Good Practice to have currentPos value same as targetPos to avoid conflicts in initial start of loop
    public double currentVelocity = 0.0;


    public static double MAX_VELOCITY = 0.05; // Maximum velocity (position increment per loop)
    public static double ACCELERATION = 0.015; // Acceleration rate
    public static double DECELERATION_DISTANCE = 0.2; // Distance to start deceleration

    public enum SHOLDER {
        AUTOINIT,
        INIT,
        INIT1,
        PICK1,
        PICK,
        DISCARD,
        TRANSFER,
    }

    public enum ROLLER {
        INTAKE,
        TRANSFER,
        THROW,
        OFF,
    }

    public enum EXTENDER {
        INIT,
        EXTEND1,
        EXTEND2,
        AUTOSAMPLE,
        AUTOSAMPLE1,
        AUTOSAMPLE2,
        AUTOSAMPLE3,
    }

    public enum Swipe{
        OPEN,
        CLOSE
    }

    public intake_ss(RobotHardware robot) {
        this.robot = robot;
    }

    public void updateState(SHOLDER state) {
        this.sholder = state;
        switch (state) {
            case AUTOINIT:
                ServoARMINIT(globals.sholderAUTOInit);
                break;
            case INIT:
                ServoARMINIT(globals.sholderInit);
                break;
            case INIT1:
                ServoARMINIT(globals.sholderInit1);
                break;
            case PICK1:
                ServoArm(globals.sholderSamplePick1);
                break;
            case PICK:
                ServoArm(globals.sholderSamplePick);
                break;
            case DISCARD:
                ServoArm(globals.SampleThrow);
                break;
            case TRANSFER:
                ServoArm(globals.sholderTransfer);
                break;
        }
    }

    public void updateState(ROLLER state) {
        this.roller = state;
        switch (state) {
            case INTAKE:
                Roller(0.4);
                break;
            case TRANSFER:
                Roller(-1);
                break;
            case THROW:
                Roller(-1);
                break;
            case OFF:
                Roller(0);
                break;
        }
    }

    public void updateState(EXTENDER state) {
        this.extender = state;
        switch (state) {
            case INIT:
                extender(0, 1);
                break;
            case EXTEND1:
                extender(300, 0.65);
                break;
            case EXTEND2:
                extender(720, 0.65);
                break;
            case AUTOSAMPLE:
                extender(500, 0.65);
                break;
            case AUTOSAMPLE1:
                extender(600, 0.5);
                break;
            case AUTOSAMPLE2:
                extender(400, 0.65);
                break;
            case AUTOSAMPLE3:
                extender(650, 0.65);
                break;

        }
    }

    public void updateState(Swipe state) {
        this.swipe = state;
        switch (state) {
            case OPEN:
                robot.swipe.setPosition(globals.swipeOpen);
                break;
            case CLOSE:
                robot.swipe.setPosition(globals.swipeClose);
                break;

        }
    }

//    public void ServoArm(double pose){
//        robot.intk_lshoulder.setPosition(pose);
//        robot.intk_rshoulder.setPosition(pose);
//    }

    //Call this method inside loop
    public void updateSholder() {
        double distanceToTarget = Math.abs(armTargetPosition - currentPosition);

        // Determine acceleration/deceleration based on distance to target
        if (distanceToTarget < DECELERATION_DISTANCE) {
            currentVelocity = Math.max(ACCELERATION, currentVelocity - ACCELERATION);
        } else if (currentVelocity < MAX_VELOCITY) {
            currentVelocity = Math.min(MAX_VELOCITY, currentVelocity + ACCELERATION);
        }

        // Update current position based on velocity and move towards target
        if (currentPosition < armTargetPosition) {
            currentPosition = Math.min(currentPosition + currentVelocity, armTargetPosition);
        } else if (currentPosition > armTargetPosition) {
            currentPosition = Math.max(currentPosition - currentVelocity, armTargetPosition);
        }

        // Apply the calculated position to the specimenArm
        setSpecimenArm(currentPosition);
    }

    //SETTING ARM TARGET-POSITION
    public void ServoArm(double position) {
        this.armTargetPosition = position;
    }

    public void ServoARMINIT(double position) {
        robot.intk_lshoulder.setPosition(position);
        robot.intk_rshoulder.setPosition(position);
    }

    //TODO======================================= SETTER METHODS ======================================
    public void setSpecimenArm(double pos) {
        robot.intk_lshoulder.setPosition(pos);
        robot.intk_rshoulder.setPosition(pos);
    }


    public void Roller(double pose) {
        robot.rollerR.setPower(pose);
        robot.rollerL.setPower(-pose);
    }

    public void extender(int targetPos, double pow) {
        robot.extender.setTargetPosition(targetPos);
        robot.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extender.setTargetPositionTolerance(10);
        robot.extender.setPower(pow);
    }

    public Action iextendCommand(EXTENDER state) {
        return new InstantAction(() -> updateState(state));
    }

    public Action iSholderCommand(SHOLDER state) {
        return new InstantAction(() -> updateState(state));
    }

    public Action iRollerCommand(ROLLER state) {
        return new InstantAction(() -> updateState(state));
    }

    public Action iswipe(Swipe state) {
        return new InstantAction(() -> updateState(state));
    }

    public void resetExtender() {
        robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
