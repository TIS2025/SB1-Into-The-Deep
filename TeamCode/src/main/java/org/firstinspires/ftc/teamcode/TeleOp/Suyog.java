package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name=" Suyog Side Roller", group="TeleOp")
public class Suyog extends LinearOpMode {
    public double position = 0;
    public static double servoPower = -1;


    public static double dcval = 0.8;
    public static double gain = 50;
    public static double amp = 2000;

    public static double redHigh = 30;
    public static double redLow = 15;
    public static double blueHigh = 240;
    public static double blueLow = 200;
    public static double yellowLow = 60;
    public static double yellowHigh = 90;
    public static double satLimit = 0.9;
    public static double distLimit = 10;

    public enum Color{
        RED,
        YELLOW,
        BLUE,
        NONE
    }

    Color sampleColor = Color.NONE;

    public static int intakeItem = 0;
    // 0 --> Nothing
    // 1 --> Red
    // 2 --> Blue
    // 3 --> Blue
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor and servo
        // Declare motor and servo
        DcMotorEx dcMotor = hardwareMap.get(DcMotorEx.class, "Motor");
        CRServo servoMotor = hardwareMap.get(CRServo.class, "servo");

        String color = null;


        // Set the motor's direction (optional, depending on motor setup)
        dcMotor.setDirection(DcMotor.Direction.REVERSE);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        NormalizedRGBA rgba;
        float[] hsv;
        double distance;

        boolean COLOR_FLAG;

        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        colorSensor.setGain((float)gain);
        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT); // Set as an input device

        double power = 0.6;

        Gamepad C1 = new Gamepad();
        Gamepad P1 = new Gamepad();

        Gamepad C2 = new Gamepad();
        Gamepad P2 = new Gamepad();

//        Trajectory pickupSample = TrajectoryActionBuilder(
//
//                lift.SamplePick(),
//                        new SleepAction(200),
//                        gripper.CloseGripper()
//                );

        waitForStart();
        while (opModeIsActive()) {

            P1.copy(C1);
            C1.copy(gamepad1);

            P2.copy(C2);
            C2.copy(gamepad2);

            rgba = colorSensor.getNormalizedColors();
            hsv = rgbToHsv(rgba.red,rgba.green,rgba.blue);
            distance = colorSensor.getDistance(DistanceUnit.MM);

            COLOR_FLAG = detectColor(rgba) == Color.RED;

            if(C1.left_bumper && P1.left_bumper) servoMotor.setPower(1);
//            else servoMotor.setPower(0);
            if(C1.right_bumper && P1.right_bumper) servoMotor.setPower(-1);
//            else servoMotor.setPower(0);

            if(C1.x) servoMotor.setPower(0);

//            if(C1.dpad_up && !P1.dpad_up) power+=0.1;
//            if(C1.dpad_down && !P1.dpad_down) power-=0.1;
            if(C1.a) dcMotor.setPower(power);
            if(C1.b) dcMotor.setPower(0);

            if(COLOR_FLAG) {
                Actions.runBlocking(new SleepAction(0.1));
                dcMotor.setPower(0);
            }



//            if(beamBreaker.getState())

            //Color test conditions
//            if((hsv[0]<redHigh && hsv[0]>redLow && hsv[1]<satLimit)||
//                    (hsv[0]<blueHigh && hsv[0]>blueLow && hsv[1]<satLimit))
//            {
//                color = "Red";
//
//            }

//            beamBreaker.setMode();

            telemetry.addLine()
                    .addData("Red", rgba.red)
                    .addData("Green", rgba.green)
                    .addData("Blue", rgba.blue);
            telemetry.addLine()
                    .addData("Hue", hsv[0])
                    .addData("Saturation", hsv[1])
                    .addData("Value", hsv[2]);
            telemetry.addLine().addData("Distance",distance);

//            telemetry.addData("Servo ",servoMotor.getPosition());
            telemetry.addData("Color ",detectColor(rgba));
            telemetry.addData("Current ",dcMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Power ",dcMotor.getPower());
            telemetry.addData("Beambreaker ",beamBreaker.getState());

            telemetry.update();


//            if ((rgba.red>rgba.blue)&&(rgba.red>rgba.green) && distance <20 ){
//                intakeItem = 1;
//                telemetry.addData("red sample", null);
//                wait(200);
//                dcMotor.setPower(0);
//                servoMotor.setPower(0);
//
//            }
//            else if ((rgba.green>0.8)&&(rgba.red>0.8) && distance <20 ){
//                intakeItem = 2;
//                telemetry.addData("yellow sample", null);
//                wait(200);
//                dcMotor.setPower(0);
//                servoMotor.setPower(0);
//            }
//            else if ((rgba.blue>rgba.red)&&(rgba.blue>rgba.green) && distance <20 ){
//                intakeItem = 3;
//                telemetry.addData("blue sample", null);
//                dcMotor.setPower(power);
//                servoMotor.setPower(servoPower);
//
//            }
            // working
//            if (((rgba.red>rgba.blue)&&(rgba.red>rgba.green) && distance <20 ) || ((rgba.green>0.8)&&(rgba.red>0.4) && distance <20 )){
//                intakeItem = 1;
//                telemetry.addData("red or yello sample", null);
//                sleep(200);
//
//                dcMotor.setPower(0);
//                servoMotor.setPower(0);
//
//            }

            // -----------------------


            /// longer distance

//            if (((rgba.green>rgba.blue)&&(rgba.green<0.1) &&(rgba.blue<0.1)&& distance <20 ) || ((rgba.green>0.3)&&(rgba.red>0.2) && distance <20 )){
//                intakeItem = 1;
//                telemetry.addData("red or yello sample", null);
//                sleep(100);
//
//                dcMotor.setPower(0);
//                servoMotor.setPower(0);
//
//            }

//            if (((rgba.red>rgba.blue)&&(rgba.red>rgba.green) && distance <20 )){
//                intakeItem = 1;
//                telemetry.addData("red sample", null);
//                sleep(100);
//
//                dcMotor.setPower(0);
//                servoMotor.setPower(0);
//
//            }
//            if (((rgba.green>0.8)&&(rgba.red>0.4) && distance <20 )){
//                intakeItem = 1;
//                telemetry.addData("yello sample", null);
//                sleep(200);
//
//                dcMotor.setPower(0);
//                servoMotor.setPower(0);
//
//            }
//
//
//            else if ((rgba.blue>rgba.red)&&(rgba.blue>rgba.green) && distance <20 ){
//                intakeItem = 3;
//                telemetry.addData("blue sample", null);
//                dcMotor.setPower(power);
//                servoMotor.setPower(servoPower);
//
//            }
//            else {
////
//                if (gamepad1.dpad_right){
//                    dcMotor.setPower(power);
//                    servoMotor.setPower(servoPower);
//                    telemetry.addData("waiting for sample", null);
//                    intakeItem = 0;
//                }
//            }
//                if(intakeItem==0){
//                    dcMotor.setPower(-power);
//                    servoMotor.setPower(-1);
//
//                }
//                if(intakeItem==1){
//                    dcMotor.setPower(-power);
//                    servoMotor.setPower(-1);
//
//                }
//                if(intakeItem==2){
//                    dcMotor.setPower(-power);
//                    servoMotor.setPower(-1);
//
//                }
//                if(intakeItem==3){
//                    dcMotor.setPower(power);
//                    servoMotor.setPower(1);
//
//                }

//                if (intakeItem!=0){
//                    dcMotor.setPower(power);
//                    servoMotor.setPower(1);
//                    wait(1000);
//                }

//            }

            // Control the DC motor with the left stick (up/down movement)
//            if (gamepad1.x){
//
//                dcMotor.setPower(power);
//
//            }
//            else if (gamepad1.y){
//
//                dcMotor.setPower(-power);
//            }
//
//            // Control the servo motor with the right stick (left/right movement)
//
//           else if(gamepad1.a){
////                positioN = positioN + 0.001;
//                servoMotor.setPower(1);
//            }
//            else if(gamepad1.b){
////                positioN = positioN - 0.001;
//                servoMotor.setPower(-1);
//            }

            // Show motor and servo positions in telemetry
//            telemetry.addData("Motor Power", power);
//            telemetry.addData("Servo Position", positioN);
//            telemetry.addData("---------------------",null);
//            telemetry.addLine()
//                    .addData("R", rgba.red)
//                    .addData("G", rgba.green)
//                    .addData("B", rgba.blue);
//            telemetry.addLine().addData("Distance",distance);
//            telemetry.addData("Motor current", dcMotor.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.update();
        }
    }

    private Color detectColor(NormalizedRGBA rgba){

        float[] hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue);

        if((hsv[0]<redHigh && hsv[0]>redLow && hsv[1]<satLimit)) return Color.RED;
        else if((hsv[0]<blueHigh && hsv[0]>blueLow && hsv[1]<satLimit)) return Color.BLUE;
        else if ((hsv[0]<yellowHigh && hsv[0]>yellowLow && hsv[1]<satLimit)) return Color.YELLOW;
        else return Color.NONE;
    }

    private float[] rgbToHsv(float rNorm, float gNorm, float bNorm) {
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
}
