package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class MecanumTeleop extends LinearOpMode {
    private CRServo c1;
    private CRServo c2;
    private DcMotor arm;
    private DcMotor elevator2;
    private DcMotor elevator;
    private Servo wrist;

    public int arm_func(float x) {
        return (int) (2000*x-36);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        c1 = hardwareMap.get(CRServo.class, "c1");
        c2 = hardwareMap.get(CRServo.class, "c2");
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm = hardwareMap.get(DcMotor.class, "arm");
        elevator2 = hardwareMap.get(DcMotor.class, "elevator2");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.resetDeviceConfigurationForOpMode();
        wrist.scaleRange(0, 1);
        wrist.setPosition(0);
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            c1.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            c2.setPower(-gamepad1.left_trigger+gamepad1.right_trigger);

            if (gamepad2.left_stick_y < 0.1) {
                arm.setTargetPosition(arm_func(Math.abs(gamepad2.left_stick_y)));
            }

            if (gamepad2.right_stick_y > 0.1) {
                elevator.setPower(0.8);
                elevator2.setPower(-0.8);
            } else if (gamepad2.right_stick_y < -0.1) {
                elevator.setPower(-0.8);
                elevator2.setPower(0.8);
            } else {
                elevator.setPower(0);
                elevator2.setPower(0);
            }
            if (gamepad2.b) {
                wrist.setPosition(1);
            } else if (gamepad2.a) {
                wrist.setPosition(0);
            } else if (gamepad2.y) {
                wrist.setPosition(0.3);
            }
            arm.setPower(1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
    }
}
