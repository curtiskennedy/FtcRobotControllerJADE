package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.OpenCVBlue;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Autonomous (name = "Blue Frontstage")
public class BlueFrontstage extends LinearOpMode {

    OpenCvWebcam webcam;

    OpenCVBlue pipeline = new OpenCVBlue(telemetry);
    private DcMotor Arm, Extend = null;

    RevBlinkinLedDriver lights;

    private double ArmPower = 0.5, SlidePower = 0.4;


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
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Left
        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(22, 0, Math.toRadians(0)))
                .back(4)
                .strafeRight(14)
                .strafeRight(8)
                .forward(20)
                .back(6)
                .strafeRight(24)
                .forward(84)
                .build();
        // Middle
        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)))
                .back(7)
//                .strafeRight(15)
//                .forward(30)
                .turn(Math.toRadians(94.5))
                .strafeRight(2) //new
                .waitSeconds(5) //new
                .addTemporalMarker(() -> {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT);
                })
                .forward(90) //104
//                .strafeLeft(25)
                .strafeRight(4) // new
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
                .strafeRight(20) // new
                .forward(10) //104

                .build();
        // Right
        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, -12, Math.toRadians(0)))
                .back(22)
                .turn(Math.toRadians(93))
                .forward(96.5)
                .strafeRight(33) // new
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
                .strafeRight(8) // new
                .forward(10) //104
                .build();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        String result = pipeline.getResult();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        if(result=="LEFT") {drive.followTrajectorySequence(Left);};
        if(result=="MIDDLE") {drive.followTrajectorySequence(Middle);};
        if(result=="RIGHT") {drive.followTrajectorySequence(Right);};


        while (opModeIsActive())
        {
            telemetry.update();
            sleep(100);
        }
    }
}
