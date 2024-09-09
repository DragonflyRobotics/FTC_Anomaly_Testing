package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: TestAuto", group="Robot")
public class TestAuto extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Motor fl, fr, br, bl;
    private MecanumDrive m_drive;
//    private GamepadEx driverOp;
    private RevNewIMU imu;
    private PurePursuitCommand ppCommand;

    // create our encoders
    Motor.Encoder encoderLeft, encoderRight, encoderPerp;

    // create the odometry object
    HolonomicOdometry holOdom;

    // create the odometry subsystem
    OdometrySubsystem odometry;

    UGRectDetector UGRectDetector;

    Waypoint p1 = new StartWaypoint(0, 0);
    Waypoint p2 = new GeneralWaypoint(1, 0, 0, 0.1, 0.1, 0.1);
    Waypoint p3 = new EndWaypoint();
    Path path = new Path(p1, p2, p3);

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");

        fr.setInverted(true);

        encoderRight = fl.encoder;
        encoderLeft = fr.encoder;
        encoderPerp = bl.encoder;

//        encoderLeft.setDirection(Motor.Direction.REVERSE);
//        encoderRight.setDirection(Motor.Direction.REVERSE);
//        encoderPerp.setDirection(Motor.Direction.REVERSE);


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
//        driverOp = new GamepadEx(gamepad1);

//        UGRectDetector = new UGRectDetector(hardwareMap);
//        UGRectDetector.init();
//        path.init();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
        ppCommand = new PurePursuitCommand(
                m_drive, odometry, p1, p2, p3);
        ppCommand.schedule();
    }

    @Override
    public void loop() {
        odometry.update();
        telemetry.addData("Pose X: ", odometry.getPose().getX());
        telemetry.addData("Pose Y: ", odometry.getPose().getY());
        telemetry.addData("Heading: ", odometry.getPose().getHeading());
        telemetry.addData("IMU Heading: ", imu.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}
}
