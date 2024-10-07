package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    // Declare OpMode members
    private LinearOpMode myOpMode = null;  // pega e extends o LinearOpMode pro myOpMode, onde a gnt so vai usar ele

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive = null;  // todos os drive motors
    private Servo   armBase, armClaw, nServo2, nServo3 = null;  // normal servos do control hub (0-3)
    private CRServo cServo4, cServo5 = null;  // continuous rotation servos do control hub (4-5)

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double LOW_SERVO       =  0.0  ;
    public static final double MID_SERVO       =  0.5  ;
    public static final double HIGH_SERVO      =  1.0  ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        try {
            leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

            myOpMode.telemetry.addData("Motor Init", "Success");

            // Servo initialization
            armBase = myOpMode.hardwareMap.get(Servo.class, "ch_nServo0");
            armClaw = myOpMode.hardwareMap.get(Servo.class, "ch_nServo1");
            myOpMode.telemetry.addData("Servo Init", "Success");

        } catch (Exception e) {
            myOpMode.telemetry.addData("Hardware Error", e.getMessage());
        }

        myOpMode.telemetry.update();

        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        // Motores (drive):
        leftFrontDrive  =  myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   =  myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive =  myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  =  myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");

        // define as direções dos motores. se ta no codigo eh pq (provavelmente) ta certo
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // bota pra run os motores com encoder pra melhorar a precisão
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Normal servos:
        armBase = myOpMode.hardwareMap.get(Servo.class, "ch_nServo0");
        armClaw = myOpMode.hardwareMap.get(Servo.class, "ch_nServo1");
        nServo2 = myOpMode.hardwareMap.get(Servo.class, "ch_nServo2");
        nServo3 = myOpMode.hardwareMap.get(Servo.class, "ch_nServo3");

        // inicia servo no low position
        armBase.setPosition(LOW_SERVO);
        armClaw.setPosition(LOW_SERVO);

        // Continuous rotation servos:
        cServo4 = myOpMode.hardwareMap.get(CRServo.class, "ch_cServo4");
        cServo5 = myOpMode.hardwareMap.get(CRServo.class, "ch_cServo5");

        // so faz um update da telemetria
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Task to perform prior to shutdown (e.g., move servos to a safe position).
     * Call this method before the robot fully stops.
     */
    public void performShutdownTask() {
        myOpMode.telemetry.addData("Task", "Moving servos to safe position...");
        myOpMode.telemetry.update();

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
        myOpMode.telemetry.addData("Task Status", completed ? "Completed" : "In Progress");
        myOpMode.telemetry.update();
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
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftRearPower, rightRearPower);
        myOpMode.telemetry.update();
    }

    // Public method to set the armBase position
    public void setArmBasePosition(double position) {
        armBase.setPosition(position);
    }

    // Public method to set the armClaw position
    public void setArmClawPosition(double position) {
        armClaw.setPosition(position);
    }

    // Public method to get the current position of the armBase
    public double getArmBasePosition() {
        return armBase.getPosition();
    }

    // Public method to get the current position of the armClaw
    public double getArmClawPosition() {
        return armClaw.getPosition();
    }
}