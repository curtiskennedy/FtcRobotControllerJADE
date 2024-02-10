package org.firstinspires.ftc.teamcode;
//package org.openftc.i2cdrivers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CURTIS TEST DO NOT CLICK THIS")
public class Curtis extends OpMode {

    private String CONTROLSMSG = "" +
            "LEFT STICK: drive+strafe\n" +
            "RIGHT STICK: turn\n" +
            "TRIANGLE: slow mode\n" +
            "TOUCHPAD: hang (full power)\n" +
            "D-PAD UP/DOWN: arm\n" +
            "D-PAD LEFT/RIGHT: slides\n" +
            "BUMPERS+TRIGGERS: flippers\n" +
            "SHARE+OPTIONS: launcher";
    //Motors
    private double MAXARMPOWER = 0.5;
    private double MAXSLIDEPOWER = 0.4;
    private double FULLHANGPOWER = 1;

    private double FLIPPERSPEED = 0.01;
    private double LEFTFLIPPERSTARTPOS = 0.5;
    private  double RIGHTFLIPPERSTARTPOS = 0.5;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Arm, Slides = null;
    private Servo leftFlipper, rightFlipper, launcher;

    private double drive, strafe, turn, armPower, slidesPower, leftFlipperPosition, rightFlipperPosition = 0.0;

    ElapsedTime runtime = new ElapsedTime();
    // don't change
    double max;
    double[] speeds = new double[4];


    @Override
    public void init() {

        //Telemetry
        telemetry.addLine(">> Welcome :)");
        telemetry.addData("CONTROLS:\n", CONTROLSMSG);
        telemetry.update();

        // Initialize DcMotors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");

        Arm = hardwareMap.get(DcMotor.class, "AE");

        Slides = hardwareMap.get(DcMotor.class, "SE");

        // Init Servos
        leftFlipper = hardwareMap.get(Servo.class, "LeftC");
        rightFlipper = hardwareMap.get(Servo.class,"RightC");
        launcher = hardwareMap.get(Servo.class, "PEW");

        leftFlipper.setPosition(LEFTFLIPPERSTARTPOS);
        rightFlipper.setPosition(RIGHTFLIPPERSTARTPOS);

        //-------------------------------------------------------
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets em to back or forward
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    //Start function
    public void start() {
        //reset
        telemetry.clearAll();


        runtime.reset();
    }

    @Override
    public void loop() {

        // Set drive controls
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // Set motor power
        speeds[0] = -drive + turn + strafe;
        speeds[1] = -drive - turn - strafe;
        speeds[2] = -drive + turn - strafe;
        speeds[3] = -drive - turn + strafe;

        max = Math.abs(speeds[0]);
        for (int i = 1; i < speeds.length; ++i) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; ++i) speeds[i] /= max;
        }


        // arm
        armPower = gamepad1.dpad_up ? MAXARMPOWER : gamepad1.dpad_down ? -MAXARMPOWER : 0;


        // slides
        slidesPower = gamepad1.dpad_right ? MAXSLIDEPOWER : gamepad1.dpad_left ? -MAXSLIDEPOWER : gamepad1.touchpad ? FULLHANGPOWER : 0;

        rightFlipperPosition += (gamepad2.right_trigger >= 0.9)? FLIPPERSPEED : gamepad2.right_bumper? -FLIPPERSPEED : 0;
        leftFlipperPosition += (gamepad2.left_trigger >= 0.9)? -FLIPPERSPEED : gamepad2.left_bumper? FLIPPERSPEED : 0;




        // Set motor powers to updated power

        if (gamepad1.triangle) { // SLOW MODE
            leftFrontDrive.setPower(speeds[0] / 3);
            rightFrontDrive.setPower(speeds[1] / 3);
            leftBackDrive.setPower(speeds[2] / 3);
            rightBackDrive.setPower(speeds[3] / 3);
        } else {
            leftFrontDrive.setPower(speeds[0]);
            rightFrontDrive.setPower(speeds[1]);
            leftBackDrive.setPower(speeds[2]);
            rightBackDrive.setPower(speeds[3]);
        }

        // launcher
        if (gamepad1.options) {
            launcher.setPosition(0);
        } else if (gamepad1.share) {
            launcher.setPosition(1);
        }


        Arm.setPower(armPower);
        Slides.setPower(slidesPower);
        rightFlipper.setPosition(rightFlipperPosition);
        leftFlipper.setPosition(leftFlipperPosition);

        telemetry.addData("CONTROLS:\n", CONTROLSMSG);
//        telemetry.addData("Arm Encoder Ticks: ", Arm.getCurrentPosition());
//        telemetry.addData("Extend Encoder Ticks", Slides.getCurrentPosition());
//        telemetry.addData("LeftFlipPosition: ", leftFlipperPower);
//        telemetry.addData("RightFlipPosition", rightFlipperPower);
    }


    @Override
    public void stop() {

        // Stop all motors if no input and if gamestop
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        Arm.setPower(0);
        Slides.setPower(0);
    }
}
