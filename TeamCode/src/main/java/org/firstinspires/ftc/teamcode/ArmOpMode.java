package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arm Control OpMode", group = "Full Tests")
public class ArmOpMode extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private TelemetryHandler telemetryHandler;

    private boolean isArmUp = false;
    private boolean isClawClosed = false;

    private boolean buttonPressed1Cross = false;
    private boolean buttonPressed1Circle = false;

    @Override
    public void runOpMode() {
        robot.init();
        telemetryHandler = new TelemetryHandler(this);

        telemetryHandler.initializeDefaults();

        waitForStart();

        while (opModeIsActive()) {
            handleMovement();
            handleArmControl();
            handleClawControl();

            // Add telemetry to confirm the loop is running
            telemetryHandler.addOrUpdate("Loop Status", "Running");
            telemetryHandler.update();
        }
    }

    @SuppressLint("DefaultLocale")
    private void handleMovement() {
        robot.driveMecanum();
        telemetryHandler.addOrUpdate("Front Wheel Powers", String.format("Front left: %.2f, Front right: %.2f",
                robot.getLeftFrontPower(), robot.getRightFrontPower()));
        telemetryHandler.addOrUpdate("Back Wheel Powers", String.format("Back left: %.2f, Back right: %.2f",
                robot.getLeftBackPower(), robot.getRightBackPower()));
    }

    private void handleArmControl() {
        telemetryHandler.addOrUpdate("X Button", gamepad1.cross ? "Pressed" : "Not Pressed");

        if (gamepad1.cross && !buttonPressed1Cross) {
            if (isArmUp) {
                robot.moveArmBaseSmoothly(RobotHardware.LOW_SERVO);
                telemetryHandler.addOrUpdate("Arm", "Lowered");
            } else {
                robot.moveArmBaseSmoothly(RobotHardware.HIGH_ARM);
                telemetryHandler.addOrUpdate("Arm", "Raised");
            }
            isArmUp = !isArmUp;
            buttonPressed1Cross = true;
        } else if (!gamepad1.cross) {
            buttonPressed1Cross = false;
        }
    }

    private void handleClawControl() {
        telemetryHandler.addOrUpdate("Circle Button", gamepad1.circle ? "Pressed" : "Not Pressed");

        if (gamepad1.circle && !buttonPressed1Circle) {
            if (isClawClosed) {
                robot.moveArmClawSmoothly(RobotHardware.LOW_SERVO);
                telemetryHandler.addOrUpdate("Claw", "Opened");
            } else {
                robot.moveArmClawSmoothly(RobotHardware.HIGH_SERVO);
                telemetryHandler.addOrUpdate("Claw", "Closed");
            }
            isClawClosed = !isClawClosed;
            buttonPressed1Circle = true;
        } else if (!gamepad1.circle) {
            buttonPressed1Circle = false;
        }
    }
}