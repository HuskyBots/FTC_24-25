package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;  // Base class for OpModes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;        // Annotation for TeleOp mode

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

        // Initialize default telemetry values.
        telemetryHandler.initializeDefaults();

        // Display initial status.
        telemetryHandler.edit("Waiting for Start", "Status.Loop Status");
        telemetryHandler.display("Status.Loop Status");

        // Wait for the start button to be pressed.
        waitForStart();

        // Update the loop status to indicate that the OpMode has started.
        telemetryHandler.edit("Running", "Status.Loop Status");

        // Main control loop, runs until the OpMode is stopped.
        while (opModeIsActive()) {
            handleMovement();      // Handle driving the robot.
            handleArmControl();    // Handle arm movement.
            handleClawControl();   // Handle claw operation.

            // Update telemetry to confirm the loop is running.
            telemetryHandler.update();
        }

        // Perform shutdown tasks when the OpMode is stopped.
        robot.performShutdownTask();
    }

    /**
     * Handles the robot's movement based on gamepad inputs.
     * It calls the drive method from the RobotHardware class.
     */
    private void handleMovement() {
        // Use the robot's drive method to control mecanum wheels.
        robot.driveMecanum();
    }

    /**
     * Handles the arm control based on the 'cross' button (typically the X button) on the gamepad.
     * Toggles the arm position between raised and lowered when the button is pressed.
     */
    private void handleArmControl() {
        // Check if the 'cross' button is pressed and was not pressed in the previous loop iteration.
        if (gamepad1.cross && !buttonPressed1Cross) {
            if (isArmUp) {
                // Move the arm down smoothly to the low servo position.
                robot.moveArmBaseSmoothly(RobotHardware.LOW_SERVO);
                telemetryHandler.edit("Lowered", "Robot.Arm Status");
            } else {
                // Move the arm up smoothly to the high arm position.
                robot.moveArmBaseSmoothly(RobotHardware.HIGH_ARM);
                telemetryHandler.edit("Raised", "Robot.Arm Status");
            }
            isArmUp = !isArmUp;           // Toggle the arm state.
            buttonPressed1Cross = true;   // Set the button pressed flag to prevent repeated toggles.

            // Display the arm status.
            telemetryHandler.display("Robot.Arm Status");
        } else if (!gamepad1.cross) {
            buttonPressed1Cross = false;  // Reset the button pressed flag when the button is released.
        }
    }

    /**
     * Handles the claw control based on the 'circle' button on the gamepad.
     * Toggles the claw position between open and closed when the button is pressed.
     */
    private void handleClawControl() {
        // Check if the 'circle' button is pressed and was not pressed in the previous loop iteration.
        if (gamepad1.circle && !buttonPressed1Circle) {
            if (isClawClosed) {
                // Open the claw smoothly to the low servo position.
                robot.moveArmClawSmoothly(RobotHardware.LOW_SERVO);
                telemetryHandler.edit("Opened", "Robot.Claw Status");
            } else {
                // Close the claw smoothly to the high servo position.
                robot.moveArmClawSmoothly(RobotHardware.HIGH_SERVO);
                telemetryHandler.edit("Closed", "Robot.Claw Status");
            }
            isClawClosed = !isClawClosed;     // Toggle the claw state.
            buttonPressed1Circle = true;      // Set the button pressed flag to prevent repeated toggles.

            // Display the claw status.
            telemetryHandler.display("Robot.Claw Status");
        } else if (!gamepad1.circle) {
            buttonPressed1Circle = false;     // Reset the button pressed flag when the button is released.
        }
    }
}