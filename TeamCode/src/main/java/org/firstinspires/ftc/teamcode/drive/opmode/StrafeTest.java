package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {

    public static double DISTANCE = 50;

    // PID coefficients for the strafe controller
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Create a PIDF controller for strafing
        PIDFController pidfController = new PIDFController(pidCoefficients);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .strafeLeft(DISTANCE)
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Get the error in strafing distance
            double errorForward = drive.getLastError().vec().getX();
            double errorBackward = drive.getLastError().vec().getX();

            telemetry.addData("Error Forward", errorForward);
            telemetry.addData("Error Backward", errorBackward);
            telemetry.update();

            // Send data to the dashboard
            packet.put("Error Forward", errorForward);
            packet.put("Error Backward", errorBackward);
            dashboard.sendTelemetryPacket(packet);

            // Adjust PID coefficients based on the error (you can tune these values)
            pidCoefficients = new PIDCoefficients(1, 0, 0);

            // Use PIDF controller to set power for strafing
            double powerForward = pidfController.update(errorForward);
            double powerBackward = pidfController.update(errorBackward);

            // Use Pose2d to set the power for strafing
            drive.setDrivePower(new Pose2d(0, powerForward, 0));
            sleep(500); // Adjust sleep duration based on your needs
            drive.setDrivePower(new Pose2d(0, -powerBackward, 0));
            sleep(500); // Adjust sleep duration based on your needs
        }
    }
}
