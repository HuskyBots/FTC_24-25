package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class RobotCentricDrive extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Map<String ,Integer> pidPositions;
    DcMotor armMotor;
    DcMotor slideMotor;
    Servo armServo;
    Servo bufferServo;
    CRServo intakeServo;
    PIDController PIDController;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        pidPositions = new HashMap<>();

        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armServo = hardwareMap.servo.get("armServo");
        bufferServo = hardwareMap.servo.get("bufferServo");

        armServo.setDirection(Servo.Direction.FORWARD);
        bufferServo.setDirection(Servo.Direction.REVERSE);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.update();

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armServo.resetDeviceConfigurationForOpMode();
        armServo.resetDeviceConfigurationForOpMode();

        PIDController = new PIDController(0.07, 0.001, 0,0, org.firstinspires.ftc.teamcode.PIDController.Mode.POSITION);
        PIDController.setTolerance(50);

        pidPositions.put("SlideRecharge",250);
        pidPositions.put("SlideHigh",2900);

        pidPositions.put("ArmRecharge",290);
        pidPositions.put("ArmTake",850);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            omniDrive();
            buffer();
            slide();
            intake();
            arm();

            telemetry.update();
        }
    }

    public void omniDrive(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void intake(){
        if(gamepad2.right_trigger > 0.5){
            intakeServo.setPower(1);
        } else {
            intakeServo.setPower(0);
        }
    }

    public void arm(){
        armMotor.setPower(gamepad2.left_stick_y);

        if(gamepad2.a){
            armMotor.setPower(PIDController.calculate(pidPositions.get("ArmRecharge"), armMotor.getCurrentPosition()));
        } else if (gamepad2.b) {
            armMotor.setPower(PIDController.calculate(pidPositions.get("ArmTake"), armMotor.getCurrentPosition()));
        }

        if(gamepad2.left_bumper){
            armServo.setPosition(0);
        } else if (gamepad2.right_bumper) {
            armServo.setPosition(1);
        }

        telemetry.addData("armServo Position", armServo.getPosition());
        telemetry.addData("ArmMotor: Encoder", armMotor.getCurrentPosition());
    }

    public void slide(){
        slideMotor.setPower(gamepad2.right_stick_y);

        if(gamepad2.dpad_up){
            slideMotor.setPower(PIDController.calculate(pidPositions.get("SlideHigh"), slideMotor.getCurrentPosition()));
        } else if (gamepad2.dpad_down){
            slideMotor.setPower(PIDController.calculate(pidPositions.get("SlideRecharge"), slideMotor.getCurrentPosition()));
        }

        telemetry.addData("SlideMotor: Encoder", slideMotor.getCurrentPosition());
    }

    public void buffer(){
        if (gamepad2.dpad_right) {
            bufferServo.setPosition(1);
        } else if (gamepad2.dpad_left){
            bufferServo.setPosition(0.15);
        }

        telemetry.addData("bufferServo Position", bufferServo.getPosition());
    }
}