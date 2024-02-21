package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //Right
        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, -12, Math.toRadians(0)))
                .back(8)
                .turn(Math.toRadians(-95))
                .forward(27)

                .strafeLeft(3)

                .addTemporalMarker(() -> {
                    // arm up
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // slides out
                    Extend.setTargetPosition(632);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)

                .back(8)

                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // slides in
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower /2f);
                })
                .waitSeconds(1)

                .strafeLeft(-17)
                .forward(12)

                .build();


        //Middle
        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)))
                .back(8)
                .turn(Math.toRadians(-95))
                .forward(39)
                .strafeLeft(5)
                .waitSeconds(.5)
                .addTemporalMarker(() -> {
                    // arm up
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1f)
                .addTemporalMarker(() -> {
                    // slides out
                    Extend.setTargetPosition(1400);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .addTemporalMarker(() -> {
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })


                .waitSeconds(1)
                .back(4)
                .waitSeconds(1)
                .strafeLeft(-25)
                .forward(12)

                .waitSeconds(1)
                .build();


        //Left
        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                .back(4)
                .strafeRight(14)
                .turn(Math.toRadians(90))
                .strafeRight(9)
                .forward(20)
                .back(8)
                .turn(Math.toRadians(190))
                .forward(34)
                .strafeLeft(3)
                .addTemporalMarker(() -> {
                    // arm up
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // slides out
                    Extend.setTargetPosition(1094);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)

                .back(8)

                .addTemporalMarker(() -> {
                    // slides in
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(1)
                .build();



        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        String result = pipeline.getResult();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

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
