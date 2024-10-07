package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Arm Control OpMode", group = "Individual Tests")
public class ArmOpMode extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);

    private boolean isArmUp = false;
    private boolean isClawClosed = false;

    private boolean buttonPressed1X = false;
    private boolean buttonPressed1Circle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            handleMovement();
            handleArmControl();
            handleClawControl();

            // Add telemetry to confirm the loop is running
            telemetry.addData("Loop Running", true);
            telemetry.update();
        }
    }

    private void handleMovement() {
        robot.driveMecanum();
    }

    private void handleArmControl() {
        telemetry.addData("X Button", gamepad1.cross ? "Pressed" : "Not Pressed");

        if (gamepad1.cross && !buttonPressed1X) {
            if (isArmUp) {
                robot.setArmBasePosition(RobotHardware.LOW_SERVO);
                telemetry.addData("Arm", "Lowered");
            } else {
                robot.setArmBasePosition(RobotHardware.HIGH_SERVO);
                telemetry.addData("Arm", "Raised");
            }
            isArmUp = !isArmUp;
            buttonPressed1X = true;
            telemetry.update(); // Update telemetry immediately
        } else if (!gamepad1.cross) {
            buttonPressed1X = false;
        }
    }

    private void handleClawControl() {
        telemetry.addData("Circle Button", gamepad1.circle ? "Pressed" : "Not Pressed");

        if (gamepad1.circle && !buttonPressed1Circle) {
            if (isClawClosed) {
                robot.setArmClawPosition(RobotHardware.LOW_SERVO);
                telemetry.addData("Claw", "Opened");
            } else {
                robot.setArmClawPosition(RobotHardware.HIGH_SERVO);
                telemetry.addData("Claw", "Closed");
            }
            isClawClosed = !isClawClosed;
            buttonPressed1Circle = true;
            telemetry.update(); // Update telemetry immediately
        } else if (!gamepad1.circle) {
            buttonPressed1Circle = false;
        }
    }
}