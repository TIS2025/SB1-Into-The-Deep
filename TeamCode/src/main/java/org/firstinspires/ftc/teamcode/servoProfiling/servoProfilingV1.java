package org.firstinspires.ftc.teamcode.servoProfiling;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class servoProfilingV1 extends LinearOpMode {

    // Define your servo
    private Servo servo;
    private Servo servo2;

    // Servo constants
    public static double SERVO_INCREMENT = 0.02; // Increment size for profiling


    // Variables to hold the target and current position
    private double targetPosition = 0.5; // Target position, start from the middle
    private double currentPosition = 0.5; // Current position of the servo

    @Override
    public void runOpMode() {
        // Initialize the servo from the hardware map
        servo = hardwareMap.get(Servo.class, "ils");
        servo2 = hardwareMap.get(Servo.class, "irs");

        // Set initial position
//        servo.setPosition(currentPosition);
            setServo(currentPosition);
        // Wait for the start of the match
        waitForStart();

        // Main loop - this runs repeatedly until the match ends
        while (opModeIsActive()) {

            // Update the target position based on controller input
            if (gamepad1.a) {
                targetPosition = 0.7961; // Move to maximum position
            } else if (gamepad1.b) {
                targetPosition = 0.2361; // Move to minimum position
            }

            // Gradually move the servo towards the target position
            currentPosition = smoothMove(currentPosition, targetPosition);

            // Set the new servo position
//            servo.setPosition(currentPosition);
            setServo(currentPosition);


            // Display the positions on telemetry
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.update();

            // Short delay to allow for smooth motion
            sleep(20);
        }
    }

    // Function to smoothly move towards target position
    private double smoothMove(double current, double target) {
        if (current < target) {
            current += SERVO_INCREMENT; // Increase the position
            if (current > target) current = target; // Avoid overshooting
        } else if (current > target) {
            current -= SERVO_INCREMENT; // Decrease the position
            if (current < target) current = target; // Avoid overshooting
        }
        return current;
    }

    private void setServo(double pos){
        servo.setPosition(pos);
        servo2.setPosition(pos);
    }
}
