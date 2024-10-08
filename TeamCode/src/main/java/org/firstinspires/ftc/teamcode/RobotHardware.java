package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file defines a Java Class that performs all the setup and configuration for our robot's hardware (motors and sensors).
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    // Declare OpMode and TelemetryHandler members
    private final LinearOpMode myOpMode;  // pega e extends o LinearOpMode pro myOpMode, onde a gnt so vai usar ele
    private final TelemetryHandler telemetryHandler;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;  // todos os drive motors
    private Servo   armBase, armClaw, nServo2, nServo3 = null;  // normal servos do control hub (0-3)
    private CRServo cServo4, cServo5 = null;  // continuous rotation servos do control hub (4-5)

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public  static final double LOW_SERVO       =  0.0  ;
    public  static final double MID_SERVO       =  0.5  ;
    public  static final double HIGH_ARM        =  0.75 ;
    public  static final double HIGH_SERVO      =  1.0  ;
    private static final double SERVO_INCREMENT =  0.01 ;  // Step size for smoothing
    private static final int    SERVO_DELAY_MS  =  20   ;  // Delay between steps for smoothing
//  public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
//  public static final double ARM_UP_POWER    =  0.45 ;
//  public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetryHandler = new TelemetryHandler(opmode);
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        try {
            // Initialize drive motors
            leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

            // Initialize normal servos
            armBase = myOpMode.hardwareMap.get(Servo.class, "ch_nServo0");
            armClaw = myOpMode.hardwareMap.get(Servo.class, "ch_nServo1");
            nServo2 = myOpMode.hardwareMap.get(Servo.class, "ch_nServo2");
            nServo3 = myOpMode.hardwareMap.get(Servo.class, "ch_nServo3");

            // Initialize continuous rotation servos
            cServo4 = myOpMode.hardwareMap.get(CRServo.class, "ch_cServo4");
            cServo5 = myOpMode.hardwareMap.get(CRServo.class, "ch_cServo5");

            // Update telemetry for successful initialization
            telemetryHandler.addOrUpdate("Status", "Hardware Initialized");
            telemetryHandler.update();

        } catch (Exception e) {
            telemetryHandler.addOrUpdate("Hardware Error", e.getMessage());
            telemetryHandler.update();
        }

        myOpMode.telemetry.update();

        // Define motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to run using encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos to their starting positions
        armBase.setPosition(LOW_SERVO);
        armClaw.setPosition(LOW_SERVO);

        // Provide final telemetry update
        telemetryHandler.update();
    }

    // Method to move the arm smoothly from current position to a target position
    public void moveServoSmoothly(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();
        targetPosition = Range.clip(targetPosition, 0.0, 1.0);  // Ensure position is between 0 and 1

        while (Math.abs(targetPosition - currentPosition) > SERVO_INCREMENT) {
            if (targetPosition > currentPosition) {
                currentPosition += SERVO_INCREMENT;
            } else {
                currentPosition -= SERVO_INCREMENT;
            }
            servo.setPosition(currentPosition);
            telemetryHandler.addOrUpdate("Servo Position", String.valueOf(currentPosition));
            telemetryHandler.update();
            myOpMode.sleep(SERVO_DELAY_MS);  // Delay between steps
        }
        // Ensure the final position is exactly the target
        servo.setPosition(targetPosition);
        telemetryHandler.addOrUpdate("Final Position", String.valueOf(targetPosition));
        telemetryHandler.update();
    }

    // Public method to control arm with smoothing
    public void moveArmBaseSmoothly(double targetPosition) {
        moveServoSmoothly(armBase, targetPosition);
    }

    // Public method to control claw with smoothing
    public void moveArmClawSmoothly(double targetPosition) {
        moveServoSmoothly(armClaw, targetPosition);
    }

    /**
     * Task to perform prior to shutdown (e.g., move servos to a safe position).
     * Call this method before the robot fully stops.
     */
    public void performShutdownTask() {
        telemetryHandler.addOrUpdate("Task", "Moving servos to safe position...");
        telemetryHandler.update();

        setArmBasePosition(LOW_SERVO);
    }

    /**
     * Checks if the shutdown task has been completed.
     * In this case, it checks if the arm has reached the LOW_SERVO position.
     *
     * @return true if the task has been completed, false otherwise.
     */
    public boolean isShutdownTaskCompleted() {
        // Check if the arm has reached the target position
        double currentPosition = getArmBasePosition();
        boolean completed = Math.abs(currentPosition - LOW_SERVO) < 0.1;
        telemetryHandler.addOrUpdate("Task Status", completed ? "Completed" : "In Progress");
        telemetryHandler.update();
        return completed;
    }

    /**
     * Handles joystick inputs from the gamepad and calculates movement parameters (power, direction, and turn).
     * This method calls the controlMecanum method to move the robot based on those inputs.
     */
    public void driveMecanum() {
        // pega os valores dos joysticks
        double x    =  myOpMode.gamepad1.left_stick_x;
        double y    = -myOpMode.gamepad1.left_stick_y;
        double turn =  myOpMode.gamepad1.right_stick_x;

        // transforma o x e y (rectangular coordinates) em polar coordinates (magnitude and direction)
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        // joga tudo isso pro controlMecanum()
        controlMecanum(power, theta, turn);
    }

    /**
     * Controls the robot's mecanum wheels by calculating the motor powers needed for movement
     * based on movement inputs: power, direction (theta), and turn.
     *
     * @param power  Magnitude of movement, determining the speed of forward/backward/strafe (0.0 to 1.0).
     * @param theta  Direction of movement, calculated using atan2 from joystick inputs (in radians).
     * @param turn   Turning input (yaw), controlling the rotation of the robot (-1.0 to 1.0).
     */
    @SuppressLint("DefaultLocale")
    public void controlMecanum(double power, double theta, double turn) {
        // seno e cosseno
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);

        // evita power overflow normalizando o seno e o cosseno
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // calcula os motor powers
        double leftFrontPower = power * cos / max + turn;
        double rightFrontPower = power * sin / max - turn;
        double leftRearPower = power * sin / max + turn;
        double rightRearPower = power * cos / max - turn;

        // evita q nenhum motor chegue a um power > 1.0
        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= (power + Math.abs(turn));
            rightFrontPower /= (power + Math.abs(turn));
            leftRearPower /= (power + Math.abs(turn));
            rightRearPower /= (power + Math.abs(turn));
        }

        // Send the calculated motor powers to the motors
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftRearPower);
        rightBackDrive.setPower(rightRearPower);

        // display a telemetria
        telemetryHandler.addOrUpdate("Front left/Right", String.format("%4.2f, %4.2f", leftFrontPower, rightFrontPower));
        telemetryHandler.addOrUpdate("Back left/Right", String.format("%4.2f, %4.2f", leftRearPower, rightRearPower));
        telemetryHandler.update();
    }

    // Public method to set the armBase position
    public void setArmBasePosition(double position) {
        armBase.setPosition(position);
    }

    // Public method to set the armClaw position
    public void setArmClawPosition(double position) {
        armClaw.setPosition(position);
    }

    /*
     * Getter methods
     */
    public double getArmBasePosition() {
        return armBase.getPosition();
    }

    public double getArmClawPosition() {
        return armClaw.getPosition();
    }

    public double getLeftFrontPower() {
        return leftFrontDrive.getPower();
    }

    public double getRightFrontPower() {
        return rightFrontDrive.getPower();
    }

    public double getLeftBackPower() {
        return leftBackDrive.getPower();
    }

    public double getRightBackPower() {
        return rightBackDrive.getPower();
    }
}
