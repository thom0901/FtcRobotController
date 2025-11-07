package org.firstinspires.ftc.teamcode.mecanum.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum TeleOp", group="Drive") // <<< deze regel maakt hem zichtbaar
public class MecanumFieldOpMode extends OpMode {
    org.firstinspires.ftc.teamcode.mecanum.drive.macanumtest drive = new org.firstinspires.ftc.teamcode.mecanum.drive.macanumtest();
    double forward, strafe, rotate;

    @Override
    public void init() {
        drive.init(hardwareMap);

        telemetry.addData("Status", "Ready");
    }

    @Override
    public void loop() {
        // Joystick-inversie (vooruit = omhoog)
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // Field-relative rijden
        drive.driveFieldRelative(forward, strafe, rotate);
    }
}