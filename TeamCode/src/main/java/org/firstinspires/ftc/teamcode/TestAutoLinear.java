package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestAutoLinear extends LinearOpMode {
    private Motor fl, fr, br, bl;
    private MecanumDrive m_drive;
    private RevNewIMU imu;
    OdometrySubsystem odometry;
    Motor.Encoder encoderLeft, encoderRight, encoderPerp;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");

        fr.setInverted(true);

        encoderRight = fl.encoder;
        encoderLeft = fr.encoder;
        encoderPerp = bl.encoder;

        encoderLeft.setDistancePerPulse(Constants.OdomConstants.TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(Constants.OdomConstants.TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(Constants.OdomConstants.TICKS_TO_INCHES);

    }
}
