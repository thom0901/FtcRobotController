package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    /**
     * Constructor: Initializes the drive system using the HardwareMap.
     */
    public MecanumDrive(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        // Attempt to retrieve all hardware components.
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right");
        backRightMotor = hwMap.get(DcMotor.class, "back_right");

        // --- Defensive Programming: Check if motor exists before setting parameters ---

        if (frontLeftMotor != null) {
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backLeftMotor != null) {
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (frontRightMotor != null) {
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backRightMotor != null) {
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Initialize IMU
        imu = hwMap.get(IMU.class, "imu");

        // --- Null Check Added Here for the IMU ---
        if (imu != null) {
            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            );
            imu.initialize(new IMU.Parameters(RevOrientation));
            imu.resetYaw();
        }
    }

    /**
     * Main method to control the drive train using driver input and speed scaling.
     * @param gamepad The Gamepad object (gamepad1)
     * @param speedScalar The factor (e.g., 0.3 or 1.0) to scale the speed by.
     */
    public void driveByGamepads(Gamepad gamepad, double speedScalar) {
        // Map joystick input
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;

        // Apply scaling factor
        forward *= speedScalar;
        strafe *= speedScalar;
        rotate *= speedScalar;

        // Pass scaled, raw input to the field-relative system
        driveFieldRelative(forward, strafe, rotate);

        // Reset the field-centric direction on Y button press
        if (gamepad.left_stick_button) {
            resetFieldCentricReference();
        }
    }

    /**
     * Calculates and sets motor power based on robot-centric vectors.
     */
    public void drive(double forward, double strafe, double rotate) {
        // Core Mecanum Formula
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalization: Scales all powers down if the combined vector exceeds 1.0.
        // This maintains the intended direction at maximum velocity.
        double max = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        // Set power only if the motor exists
        if (frontLeftMotor != null) frontLeftMotor.setPower(frontLeftPower);
        if (backLeftMotor != null) backLeftMotor.setPower(backLeftPower);
        if (frontRightMotor != null) frontRightMotor.setPower(frontRightPower);
        if (backRightMotor != null) backRightMotor.setPower(backRightPower);
    }

    /**
     * Calculates motor powers for Field-Relative driving.
     */
    public void driveFieldRelative(double forward, double strafe, double rotate) {
        if (imu == null) {
            this.drive(forward, strafe, rotate);
            return;
        }

        // Convert to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Rotate angle by robot heading
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public void resetFieldCentricReference() {
        if (imu != null) {
            imu.resetYaw();
        }
    }
}