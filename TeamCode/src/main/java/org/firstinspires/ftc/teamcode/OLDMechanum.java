package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp(name="Robot: Teleop Mecanum", group="Robot")
public class OLDMechanum extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Motor fl, fr, br, bl;
    private MecanumDrive m_drive;
    private GamepadEx driverOp;
    private RevNewIMU imu;
    private Servo servo;




    // create our encoders
    Motor.Encoder encoderLeft, encoderRight, encoderPerp;

    // create the odometry object
    HolonomicOdometry holOdom;

    // create the odometry subsystem
    OdometrySubsystem odometry;

    UGRectDetector UGRectDetector;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");
        servo = hardwareMap.get(Servo.class, "servo");

        fr.setInverted(true);

        encoderRight = fl.encoder;
        encoderLeft = fr.encoder;
        encoderPerp = bl.encoder;


        encoderLeft.setDistancePerPulse(Constants.OdomConstants.TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(Constants.OdomConstants.TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(Constants.OdomConstants.TICKS_TO_INCHES);

        holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                Constants.OdomConstants.TRACKWIDTH, Constants.OdomConstants.CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);

        imu = new RevNewIMU(hardwareMap, "imu");
        imu.init();

        m_drive = new MecanumDrive(fl, fr, bl, br);
        driverOp = new GamepadEx(gamepad1);

//        UGRectDetector = new UGRectDetector(hardwareMap);
//        UGRectDetector.init();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        m_drive.driveFieldCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), imu.getHeading());
        odometry.update();
        telemetry.addData("Pose X: ", odometry.getPose().getX());
        telemetry.addData("Pose Y: ", odometry.getPose().getY());
        telemetry.addData("Heading: ", odometry.getPose().getHeading());
        telemetry.addData("IMU Heading: ", imu.getHeading());
        telemetry.update();
        servo.setPosition(driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
    }

    @Override
    public void stop() {}
}
