package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "RedBackstage")
public class RedBackstage extends LinearOpMode {

    OpenCvWebcam webcam;

    OpenCVRed pipeline = new OpenCVRed(telemetry);
    private DcMotor Arm, Extend = null;

    private double ArmPower = 0.5, SlidePower = 0.4;
    public Servo lancher, leftFlipper, rightFlipper = null;


    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotor.class, "AE");
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setDirection(DcMotor.Direction.REVERSE);

        Extend = hardwareMap.get(DcMotor.class, "SE");
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setDirection((DcMotor.Direction.REVERSE));


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                leftFlipper = hardwareMap.get(Servo.class, "LeftC");
                rightFlipper = hardwareMap.get(Servo.class,"RightC");

            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        telemetry.addLine("Waiting for start");
        telemetry.update();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24.5, 5.5), Math.toRadians(47))
                .back(12f)
                .turn(Math.toRadians(-140))
                .forward(34)
                .strafeLeft(16)
                .forward(2)
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
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })

                .waitSeconds(1)
                .back(8)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .strafeRight(30.937638637376f)
                .forward(8.98765434567)
                .build();

        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)))
                .back(8)
                .turn(Math.toRadians(-95))
                .forward(38)
                .strafeLeft(3)
                .forward(2)
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
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .back(8)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })

                .waitSeconds(1)
                .strafeRight(25)
                .forward(15)
                .build();
//test
        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(26, -14, Math.toRadians(0)))
                .back(9)
                .turn(Math.toRadians(-95))
                .forward(25)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(900);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(1300);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })

                .waitSeconds(1)
                .back(8)
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })

                .waitSeconds(1)
                .strafeRight(16)
                .forward(15)
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

        if (result == "LEFT") {
            drive.followTrajectorySequence(Left);
        }
        if (result == "MIDDLE") {
            drive.followTrajectorySequence(Middle);
        }
        if (result == "RIGHT") {
            drive.followTrajectorySequence(Right);
        }


        while (opModeIsActive()) {

            telemetry.update();
            sleep(100);
        }
    }
}
