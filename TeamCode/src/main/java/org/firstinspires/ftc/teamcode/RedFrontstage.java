package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.OpenCVRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
//میان ما متعهد شده است
@Autonomous (name = "Red Frontstage")
public class RedFrontstage extends LinearOpMode {
    OpenCvWebcam webcam;

    RevBlinkinLedDriver lights;
    OpenCVRed pipeline = new OpenCVRed(telemetry);
    private DcMotor Arm, Extend = null;
    private double ArmPower = 0.5, SlidePower = 0.4;
    public Servo lancher, leftFlipper, rightFlipper = null;

    @Override
    public void runOpMode()
    {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        Arm = hardwareMap.get(DcMotor.class, "AE");
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setDirection(DcMotor.Direction.REVERSE);

        Extend = hardwareMap.get(DcMotor.class, "SE");
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setDirection(DcMotor.Direction.REVERSE);
/*
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                leftFlipper = hardwareMap.get(Servo.class, "LeftC");
                rightFlipper = hardwareMap.get(Servo.class,"RightC");

            }

            @Override
            public void onError(int errorCode) {telemetry.clearAll(); telemetry.addLine("Camera failed to open");}
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Left
        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(27, -8), Math.toRadians(-47))
                .back(14.142135623730950488016887242097)
                .turn(Math.toRadians(-47))
                .strafeRight(14)
                .waitSeconds(5)
                .forward(60)
                .strafeLeft(16)
                .forward(30.5)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(800);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(1394);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(550);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .back(8)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })

                .build();
        //Middle
        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)))
                .back(7)
//                .strafeRight(15)
//                .forward(30)
                .turn(Math.toRadians(-94.5))
                .strafeLeft(2) //new
                .addTemporalMarker(() -> {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);

                })
                .waitSeconds(1) //new
                .addTemporalMarker(() -> {

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);


                })
                .waitSeconds(8)
                .addTemporalMarker(() -> {

                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                })


                .forward(89)
                .strafeLeft(3)//104
//                .strafeLeft(25)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(1094);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(550);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .back(8)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .build();
        //Right
        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, 10, Math.toRadians(0)))
                .back(22)
                .turn(Math.toRadians(-92))
                .forward(65)
                .waitSeconds(0.1)
                .turn(Math.toRadians(92))
                .forward(31) // new
                .waitSeconds(0.1)
                .turn(Math.toRadians(-96))
                .forward(36)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(1094);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(550);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .back(6) // new

                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })

                .build();
        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        String result = pipeline.getResult();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        rightFlipper.setPosition(-2);
        leftFlipper.setPosition(2);
        if(result=="RIGHT") {drive.followTrajectorySequence(Left);};
        if(result=="MIDDLE") {drive.followTrajectorySequence(Middle);};
        if(result=="LEFT") {drive.followTrajectorySequence(Right);};

        while (opModeIsActive())
        {
            telemetry.update();
            sleep(100);
        }
    }
}
