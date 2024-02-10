//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Vision.OpenCVBlue;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//
//@Autonomous(name = "Blue Backstage")
//public class BlueBackstageExample extends LinearOpMode {
//    OpenCvWebcam webcam;
//    OpenCVBlue pipeline = new OpenCVBlue(telemetry);
//
//
//    @Override
//    public void runOpMode() {
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(pipeline);
//        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//
//
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//
//        waitForStart();
//
//        String result = pipeline.getResult();
//        webcam.stopStreaming();
//        webcam.closeCameraDevice();
//
//        if (result == "LEFT") Left();
//        if (result == "MIDDLE") Middle();
//        if (result == "RIGHT") Right();
//
//        while (opModeIsActive()) {
//            telemetry.update();
//            sleep(100);
//        }
//    }
//
//    void Left() {
//
//    }
//
//    void Middle() {
//
//    }
//
//    void Right() {
//
//    }
//}
