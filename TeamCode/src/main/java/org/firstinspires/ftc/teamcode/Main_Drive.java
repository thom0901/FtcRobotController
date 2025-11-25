package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings("unused") // We will not get the "Unused function" warning if it is causing problems remove it

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Main Drive", group = "Iterative OpMode")

// This what you use in the TeleOp
public class Main_Drive extends OpMode {

    MecanumDrive drive;
    //private DcMotor coreHex;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);

        //coreHex = hardwareMap.get(DcMotor.class, "CoreHex");
        //coreHex.setDirection(DcMotor.Direction.REVERSE);
        //coreHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // If the right bumper is held, use slow speed (0.3). Otherwise, use full speed (1.0).
        double speedScalar = gamepad1.right_bumper ? 0.3 : 1.0;

        drive.driveByGamepads(gamepad1, speedScalar);

        // CoreHex motor control
        //if (gamepad1.a) {
        //    coreHex.setPower(1.0);
        //} else {
        //    coreHex.setPower(0.0);
        //}


        telemetry.addData("Speed Mode", gamepad1.right_bumper ? "SLOW" : "FAST");

        //telemetry.addData("CoreHex Power", gamepad1.a ? "ON" : "OFF");
        telemetry.update();
    }


}