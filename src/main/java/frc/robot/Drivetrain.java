package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
    // 3 meters per second.
    public static final double kMaxSpeed = 3.0;
    // 1/2 rotation per second.
    public static final double kMaxAngularSpeed = Math.PI;

    private static final double kTrackWidth = 0.381 * 2;
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = -4096;

    private final Field2d m_fieldSim = new Field2d();

    private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);
    
    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
    
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // temp - need a real number!
    private final int countsPerRev = 1000;
    private final RelativeEncoder frontLeftEncoder = Chassis.fLeft.getAlternateEncoder(countsPerRev);
    private final RelativeEncoder frontRightEncoder = Chassis.fRight.getAlternateEncoder(countsPerRev);
    private final RelativeEncoder backLeftEncoder = Chassis.bLeft.getAlternateEncoder(countsPerRev);
    private final RelativeEncoder backRightEncoder = Chassis.bRight.getAlternateEncoder(countsPerRev);

    private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(
          m_drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);
          
    // Gains are for example purposes only - must be determined for your own
    // robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    
    public Drivetrain() {

        SmartDashboard.putData("Field", m_fieldSim);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
        //TODO - determine how to getRate() from RelativeEncoder, maybe use getVelocity() ?
        // double leftOutput =
        //     m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
        // double rightOutput =
        //     m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
        double leftOutput =
            m_leftPIDController.calculate(frontLeftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        double rightOutput =
            m_rightPIDController.calculate(frontRightEncoder.getVelocity(), speeds.rightMetersPerSecond);

        // this is not changing, the method is not being called.
        // System.out.println("leftOutput=" + leftOutput + ", rightOutput=" + rightOutput);

        // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        // m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        Chassis.driveSpd(leftOutput+leftFeedforward, rightOutput + rightFeedforward);
    }

    public void drive(double xSpeed, double rot) {
        setSpeeds(m_kinematics.toWheelSpeeds((new ChassisSpeeds(xSpeed, 0, rot))));
    }

    public void updateOdometry() {
        double leftDistance = frontLeftEncoder.getPosition() * kWheelRadius * 2 * Math.PI / kEncoderResolution;
        double rightDistance = frontRightEncoder.getPosition() * kWheelRadius * 2 * Math.PI /kEncoderResolution;

        //debug
        System.out.println("fLeft encoder: " + frontLeftEncoder.getPosition() +
        ", fRight encoder: " + frontRightEncoder.getPosition() +
        ", bLeft encoder: " + backLeftEncoder.getPosition() +
        ", bRight encoder: " + backRightEncoder.getPosition() 
        );
        // System.out.println("position fLeft: " + Chassis.fLeft.getPosition());

        // these values don't change
        // System.out.println("leftDistance: " + leftDistance + ", rightDistance: " + rightDistance);

        // heading is changing with left joystick moving up/down, because of line 142
        // System.out.println("heading: " + m_gyro.getRotation2d());

        m_odometry.update(m_gyro.getRotation2d(), leftDistance, rightDistance);
    }

    public void resetOdometry(Pose2d pose) {
        Chassis.fLeft.reset();
        Chassis.fRight.reset();
        Chassis.bLeft.reset();
        Chassis.bRight.reset();
        m_drivetrainSimulator.setPose(pose);
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

     /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        // m_drivetrainSimulator.setInputs(
        //     m_leftLeader.get() * RobotController.getInputVoltage(),
        //     -m_rightLeader.get() * RobotController.getInputVoltage());
        // double voltage = RobotController.getInputVoltage();
        // this is always 12 volts
        //System.out.println("voltage="+voltage);

        // this is changing with left joystick
        // System.out.println(String.format("fLeft: %2.1f, fRight: %2.1f, bLeft: %2.1f, bRight: %2.1f",
        //     Chassis.fLeft.get(), Chassis.fRight.get(), Chassis.bLeft.get(),Chassis.bRight.get()));

        // m_drivetrainSimulator.setInputs(
        //     Chassis.fLeft.get() * RobotController.getInputVoltage(),
        //     -Chassis.fRight.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.setInputs(
            Chassis.fLeft.get() * RobotController.getInputVoltage(),
            Chassis.fRight.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);

        // m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        // m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        // m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

        //this changes when left joystick is moved up and down
        // System.out.println("angle: " + -m_drivetrainSimulator.getHeading().getDegrees());

        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
        
    }

    public void periodic() {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

}
