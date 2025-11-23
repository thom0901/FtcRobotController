package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MecanumFieldOrientatedOpMode", group = "Iterative Opmode")
public class MecanumFieldOrientatedOpMode extends OpMode {

    org.firstinspires.ftc.teamcode.mecanum.drive.MecanumDrive drive =
            new org.firstinspires.ftc.teamcode.mecanum.drive.MecanumDrive();

    // ➤ Core Hex motor als DC-motor
    private DcMotor coreHex;

    double forward, strafe, rotate;

    @Override
    public void init() {
        drive.init(hardwareMap);

        // ➤ Core Hex motor uit hardwaremap halen
        // Zorg dat hij in Robot Config als "Motor" staat met de naam: coreHex
        coreHex = hardwareMap.get(DcMotor.class, "corehex");
        coreHex.setDirection(DcMotor.Direction.REVERSE);


        // Optioneel: remmen wanneer power = 0 (meestal handig)
        coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // Mecanum besturing
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);

        // ➤ Core Hex draait constant wanneer A wordt ingedrukt
        if (gamepad1.a) {
            coreHex.setPower(1 ); // constante power (kan je aanpassen)
        } else {
            coreHex.setPower(0);
        }


        telemetry.addData("CoreHex Power", gamepad1.a ? "1.0" : "0");
    }
}
