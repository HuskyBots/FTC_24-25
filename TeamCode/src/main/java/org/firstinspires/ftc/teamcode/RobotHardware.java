package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo; // Continuous Rotation Servo
import com.qualcomm.robotcore.hardware.DcMotor; // DC Motor
import com.qualcomm.robotcore.hardware.Servo;   // Standard Servo
import com.qualcomm.robotcore.util.Range;       // Utility for value clamping

/**
 * This class handles all the setup and configuration for the robot's hardware components,
 * such as motors and servos. It abstracts the hardware initialization and control,
 * allowing the main OpMode code to interact with the robot hardware through simple method calls.
 * This encapsulation promotes cleaner code and easier maintenance.
 */
public class RobotHardware {

    // Reference to the OpMode instance that created this RobotHardware object.
    private final LinearOpMode myOpMode;

    // Handler for managing telemetry updates (displaying data on the driver station).
    private final TelemetryHandler telemetryHandler;

    // Declare motor objects for the robot's drive system (all initialized to null).
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare standard servo objects connected to the Control Hub (ports 0-3).
    private Servo armBase = null;
    private Servo armClaw = null;
    private Servo nServo2 = null;
    private Servo nServo3 = null;

    // Declare continuous rotation servo objects connected to the Control Hub (ports 4-5).
    private CRServo cServo4 = null;
    private CRServo cServo5 = null;

    // Define constants for servo positions and movement increments.
    public static final double LOW_SERVO = 0.0;       // Minimum servo position
    public static final double MID_SERVO = 0.5;       // Middle servo position
    public static final double HIGH_ARM = 0.75;       // High position for the arm servo
    public static final double HIGH_SERVO = 1.0;      // Maximum servo position

    // To review this implementation, and rather consider adding speed and power coefficients.
    private static final double SERVO_INCREMENT = 0.01; // Increment for smooth servo movement
    private static final int SERVO_DELAY_MS = 20;       // Delay between servo increments (in milliseconds)

    // Constructor that accepts a reference to the OpMode that is using this hardware class.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetryHandler = new TelemetryHandler(opmode);
    }

    /**
     * Initializes all the robot's hardware components.
     * This method should be called once during the OpMode's initialization phase.
     */
    public void init() {
        try {
            // Initialize drive motors from the hardware map using their configuration names.
            leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive   = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

            // Initialize standard servos from the hardware map.
            armBase = myOpMode.hardwareMap.get(Servo.class, "ch_nServo0");
            armClaw = myOpMode.hardwareMap.get(Servo.class, "ch_nServo1");
            nServo2 = myOpMode.hardwareMap.get(Servo.class, "ch_nServo2");
            nServo3 = myOpMode.hardwareMap.get(Servo.class, "ch_nServo3");

            // Initialize continuous rotation servos from the hardware map.
            cServo4 = myOpMode.hardwareMap.get(CRServo.class, "ch_cServo4");
            cServo5 = myOpMode.hardwareMap.get(CRServo.class, "ch_cServo5");

            // Update telemetry to indicate successful hardware initialization.
            telemetryHandler.addOrUpdate("Status", "Hardware Initialized");
            telemetryHandler.update();

        } catch (Exception e) {
            // If there is an error during initialization, display the error message.
            telemetryHandler.addOrUpdate("Hardware Error", e.getMessage());
            telemetryHandler.update();
        }

        // Configure motor directions. Reverse the direction of specific motors to match physical configuration.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to run using encoders for precise control.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos to their default starting positions.
        armBase.setPosition(LOW_SERVO);
        armClaw.setPosition(LOW_SERVO);

        // Final telemetry update after initialization.
        telemetryHandler.update();
    }

    /**
     * Moves a servo smoothly from its current position to a target position.
     * This method gradually changes the servo position in small increments to create smooth movement.
     *
     * @param servo          The servo object to move.
     * @param targetPosition The desired target position (range from 0.0 to 1.0).
     */
    @SuppressLint("DefaultLocale")
    public void moveServoSmoothly(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();
        targetPosition = Range.clip(targetPosition, 0.0, 1.0); // Ensure target position is within valid range.

        // Continue moving the servo until it reaches the target position.
        while (Math.abs(targetPosition - currentPosition) > SERVO_INCREMENT) {
            if (targetPosition > currentPosition) {
                currentPosition += SERVO_INCREMENT;
            } else {
                currentPosition -= SERVO_INCREMENT;
            }
            servo.setPosition(currentPosition);

            // Update telemetry with the current servo position.
            telemetryHandler.addOrUpdate("Servo Position", String.format("%.2f", currentPosition));
            telemetryHandler.update();

            // Pause briefly between increments for smooth movement.
            myOpMode.sleep(SERVO_DELAY_MS);
        }

        // Ensure the servo is set to the exact target position at the end.
        servo.setPosition(targetPosition);

        // Update telemetry with the final position.
        telemetryHandler.addOrUpdate("Final Position", String.format("%.2f", targetPosition));
        telemetryHandler.update();
    }

    /**
     * Moves the arm base servo smoothly to the specified target position.
     *
     * @param targetPosition The desired target position for the arm base servo (0.0 to 1.0).
     */
    public void moveArmBaseSmoothly(double targetPosition) {
        moveServoSmoothly(armBase, targetPosition);
    }

    /**
     * Moves the arm claw servo smoothly to the specified target position.
     *
     * @param targetPosition The desired target position for the arm claw servo (0.0 to 1.0).
     */
    public void moveArmClawSmoothly(double targetPosition) {
        moveServoSmoothly(armClaw, targetPosition);
    }

    /**
     * Performs tasks prior to robot shutdown, such as moving servos to safe positions.
     * This method should be called before the robot fully stops to ensure hardware is in a safe state.
     */
    public void performShutdownTask() {
        telemetryHandler.addOrUpdate("Task", "Moving servos to safe position...");
        telemetryHandler.update();

        // Move the arm base to the lowest position for safety.
        setArmBasePosition(LOW_SERVO);
    }

    /**
     * Checks if the shutdown task has been completed.
     * Specifically, it verifies if the arm base servo has reached the LOW_SERVO position.
     *
     * @return True if the shutdown task is completed, false otherwise.
     */
    public boolean isShutdownTaskCompleted() {
        // Get the current position of the arm base servo.
        double currentPosition = getArmBasePosition();
        // Determine if the position is close enough to the LOW_SERVO position.
        boolean completed = Math.abs(currentPosition - LOW_SERVO) < 0.1;

        // Update telemetry with the task status.
        telemetryHandler.addOrUpdate("Task Status", completed ? "Completed" : "In Progress");
        telemetryHandler.update();

        return completed;
    }

    /**
     * Handles joystick inputs from the gamepad to calculate movement parameters.
     * It reads the joystick values and computes the power, direction, and turning rate.
     * These values are then used to control the robot's mecanum wheels.
     */
    public void driveMecanum() {
        // Read joystick values from the gamepad.
        double x = myOpMode.gamepad1.left_stick_x;    // Strafe movement (left/right).
        double y = -myOpMode.gamepad1.left_stick_y;   // Forward/backward movement (inverted).
        double turn = myOpMode.gamepad1.right_stick_x; // Rotation (turning left/right).

        // Convert joystick inputs to polar coordinates (magnitude and direction).
        double theta = Math.atan2(y, x);              // Direction angle in radians.
        double power = Math.hypot(x, y);              // Magnitude of movement.

        // Call the method to control the mecanum wheels with calculated parameters.
        controlMecanum(power, theta, turn);
    }

    /**
     * Controls the robot's mecanum wheels by calculating and setting motor powers.
     * It uses the desired movement power, direction, and turning input to compute individual motor powers.
     *
     * @param power Magnitude of movement (0.0 to 1.0).
     * @param theta Direction of movement in radians.
     * @param turn  Turning input (-1.0 to 1.0), where positive values turn right.
     */
    @SuppressLint("DefaultLocale") // Suppresses locale warnings for String.format().
    public void controlMecanum(double power, double theta, double turn) {
        // Calculate the sine and cosine components for the desired direction.
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);

        // Normalize the components to prevent power values exceeding 1.0.
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Calculate the power for each motor, including the turning component.
        double leftFrontPower = power * cos / max + turn;
        double rightFrontPower = power * sin / max - turn;
        double leftRearPower = power * sin / max + turn;
        double rightRearPower = power * cos / max - turn;

        // Normalize the motor powers if the combined power exceeds 1.0.
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower))));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightRearPower /= maxPower;
        }

        // Set the calculated power to each motor.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftRearPower);
        rightBackDrive.setPower(rightRearPower);

        // Update telemetry with the motor power values.
        telemetryHandler.addOrUpdate("Front left/Right",
                String.format("%.2f, %.2f", leftFrontPower, rightFrontPower));
        telemetryHandler.addOrUpdate("Back left/Right",
                String.format("%.2f, %.2f", leftRearPower, rightRearPower));
        telemetryHandler.update();
    }

    /**
     * Sets the position of the arm base servo to a specified value.
     *
     * @param position The desired servo position (0.0 to 1.0).
     */
    public void setArmBasePosition(double position) {
        armBase.setPosition(position);
    }

    /**
     * Sets the position of the arm claw servo to a specified value.
     *
     * @param position The desired servo position (0.0 to 1.0).
     */
    public void setArmClawPosition(double position) {
        armClaw.setPosition(position);
    }

    /*
     * Getter methods for retrieving current positions and motor powers.
     * These methods allow other classes to access the internal states in a controlled manner.
     */

    /**
     * Gets the current position of the arm base servo.
     *
     * @return The servo position (0.0 to 1.0).
     */
    public double getArmBasePosition() {
        return armBase.getPosition();
    }

    /**
     * Gets the current position of the arm claw servo.
     *
     * @return The servo position (0.0 to 1.0).
     */
    public double getArmClawPosition() {
        return armClaw.getPosition();
    }

    /**
     * Gets the current power applied to the left front drive motor.
     *
     * @return The motor power (-1.0 to 1.0).
     */
    public double getLeftFrontPower() {
        return leftFrontDrive.getPower();
    }

    /**
     * Gets the current power applied to the right front drive motor.
     *
     * @return The motor power (-1.0 to 1.0).
     */
    public double getRightFrontPower() {
        return rightFrontDrive.getPower();
    }

    /**
     * Gets the current power applied to the left back drive motor.
     *
     * @return The motor power (-1.0 to 1.0).
     */
    public double getLeftBackPower() {
        return leftBackDrive.getPower();
    }

    /**
     * Gets the current power applied to the right back drive motor.
     *
     * @return The motor power (-1.0 to 1.0).
     */
    public double getRightBackPower() {
        return rightBackDrive.getPower();
    }
}
