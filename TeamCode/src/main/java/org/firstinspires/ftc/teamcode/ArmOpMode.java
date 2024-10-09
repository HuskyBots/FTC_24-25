package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;  // Base class for OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;       // Annotation for TeleOp mode

/**
 * This OpMode class controls the robot's arm and claw mechanisms, as well as driving using mecanum wheels.
 * It utilizes the RobotHardware class to interact with the robot's hardware components.
 * The class handles user inputs from the gamepad to perform actions like moving the arm up or down,
 * opening or closing the claw, and driving the robot.
 */
@TeleOp(name = "Arm Control OpMode", group = "Full Tests")
public class ArmOpMode extends LinearOpMode {

    // Create an instance of the RobotHardware class to access hardware components and methods.
    private final RobotHardware robot = new RobotHardware(this);

    // TelemetryHandler for displaying data on the driver station.
    private TelemetryHandler telemetryHandler;

    // Flags to track the state of the arm and claw.
    private boolean isArmUp = false;
    private boolean isClawClosed = false;

    // Flags to track button press states to handle toggle actions.
    private boolean buttonPressed1Cross = false;   // For the 'cross' button (X button)
    private boolean buttonPressed1Circle = false;  // For the 'circle' button (B button)

    /**
     * This method is called when the OpMode is selected from the driver station.
     * It initializes the robot hardware and runs the main control loop.
     */
    @Override
    public void runOpMode() {
        // Initialize the robot hardware.
        robot.init();

        // Initialize the telemetry handler.
        telemetryHandler = new TelemetryHandler(this);

        // Initialize default telemetry values if needed.
        telemetryHandler.initializeDefaults();

        // Wait for the start button to be pressed.
        waitForStart();

        // Main control loop, runs until the OpMode is stopped.
        while (opModeIsActive()) {
            handleMovement();      // Handle driving the robot.
            handleArmControl();    // Handle arm movement.
            handleClawControl();   // Handle claw operation.

            // Update telemetry to confirm the loop is running.
            telemetryHandler.addOrUpdate("Loop Status", "Running");
            telemetryHandler.update();
        }
    }

    /**
     * Handles the robot's movement based on gamepad inputs.
     * It calls the drive method from the RobotHardware class and updates telemetry.
     */
    @SuppressLint("DefaultLocale") // Suppresses locale warnings for String.format().
    private void handleMovement() {
        // Use the robot's drive method to control mecanum wheels.
        robot.driveMecanum();

        // Update telemetry with the current motor power levels.
        telemetryHandler.addOrUpdate("Front Wheel Powers",
                String.format("Front left: %.2f, Front right: %.2f",
                        robot.getLeftFrontPower(), robot.getRightFrontPower()));
        telemetryHandler.addOrUpdate("Back Wheel Powers",
                String.format("Back left: %.2f, Back right: %.2f",
                        robot.getLeftBackPower(), robot.getRightBackPower()));
    }

    /**
     * Handles the arm control based on the 'cross' button (typically the X button) on the gamepad.
     * Toggles the arm position between raised and lowered when the button is pressed.
     */
    private void handleArmControl() {
        // Update telemetry with the state of the 'cross' button.
        telemetryHandler.addOrUpdate("X Button", gamepad1.cross ? "Pressed" : "Not Pressed");

        // Check if the 'cross' button is pressed and was not pressed in the previous loop iteration.
        if (gamepad1.cross && !buttonPressed1Cross) {
            if (isArmUp) {
                // Move the arm down smoothly to the low servo position.
                robot.moveArmBaseSmoothly(RobotHardware.LOW_SERVO);
                telemetryHandler.addOrUpdate("Arm", "Lowered");
            } else {
                // Move the arm up smoothly to the high arm position.
                robot.moveArmBaseSmoothly(RobotHardware.HIGH_ARM);
                telemetryHandler.addOrUpdate("Arm", "Raised");
            }
            isArmUp = !isArmUp;           // Toggle the arm state.
            buttonPressed1Cross = true;   // Set the button pressed flag to prevent repeated toggles.
        } else if (!gamepad1.cross) {
            buttonPressed1Cross = false;  // Reset the button pressed flag when the button is released.
        }
    }

    /**
     * Handles the claw control based on the 'circle' button (typically the B button) on the gamepad.
     * Toggles the claw position between open and closed when the button is pressed.
     */
    private void handleClawControl() {
        // Update telemetry with the state of the 'circle' button.
        telemetryHandler.addOrUpdate("Circle Button", gamepad1.circle ? "Pressed" : "Not Pressed");

        // Check if the 'circle' button is pressed and was not pressed in the previous loop iteration.
        if (gamepad1.circle && !buttonPressed1Circle) {
            if (isClawClosed) {
                // Open the claw smoothly to the low servo position.
                robot.moveArmClawSmoothly(RobotHardware.LOW_SERVO);
                telemetryHandler.addOrUpdate("Claw", "Opened");
            } else {
                // Close the claw smoothly to the high servo position.
                robot.moveArmClawSmoothly(RobotHardware.HIGH_SERVO);
                telemetryHandler.addOrUpdate("Claw", "Closed");
            }
            isClawClosed = !isClawClosed;     // Toggle the claw state.
            buttonPressed1Circle = true;      // Set the button pressed flag to prevent repeated toggles.
        } else if (!gamepad1.circle) {
            buttonPressed1Circle = false;     // Reset the button pressed flag when the button is released.
        }
    }
}