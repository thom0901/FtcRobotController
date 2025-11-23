package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamOpmode extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        // vision portal updaten
        aprilTagWebcam.update();

        // specifieke tag ophalen
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        // telemetry tonen
        aprilTagWebcam.displayDetectionTelemetry(id20);
    }
}
