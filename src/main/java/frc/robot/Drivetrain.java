package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
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
    // private static final double kWheelRadius = 0.0508;

    private static final double kWheelRadius = 0.1508; // 6 inches
    //TODO - need to make this match maxRPM in Chassis.java
    private static final int kEncoderResolution = -5700;

    private final Field2d m_fieldSim = new Field2d();

    // private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
    // private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);
    
    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
    
    // private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // temp - need a real number!
    private final int countsPerRev = 4000;
    
    private final RelativeEncoder frontLeftEncoder = Chassis.fLeft.getEncoder();
    private final RelativeEncoder frontRightEncoder = Chassis.fRight.getEncoder();

    private double mass = 54; // kilograms
    private double momemtOfInertia = 3; //6; // kgmm

    // private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
    //   LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    // private final DifferentialDrivetrainSim m_drivetrainSimulator =
    //   new DifferentialDrivetrainSim(
    //       m_drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

    // private final DifferentialDrivetrainSim m_drivetrainSimulator =
    //   new DifferentialDrivetrainSim(
    //       m_drivetrainSystem, DCMotor.getNEO(2), 8, kTrackWidth, kWheelRadius, null);

    private final DifferentialDrivetrainSim m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
               DCMotor.getNEO(2), 8, momemtOfInertia, mass,kWheelRadius,  kTrackWidth, null);
          
    // Gains are for example purposes only - must be determined for your own
    // robot!
    // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    
    public Drivetrain() {

        SmartDashboard.putData("Field", m_fieldSim);
    }

    // not used
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        // var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        // double leftOutput =
        //     m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
        // double rightOutput =
        //     m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

        // double leftOutput =
        //     Chassis.fLeftPidController.calculate(frontLeftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        // double rightOutput =
        //     Chassis.fRightPidController.calculate(frontRightEncoder.getVelocity(), speeds.rightMetersPerSecond);

        // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        // m_rightGroup.setVoltage(rightOutput + rightFeedforward);

        // Chassis.driveSpd(leftOutput+leftFeedforward, rightOutput + rightFeedforward);
    }

    // not used
    public void drive(double xSpeed, double rot) {
        setSpeeds(m_kinematics.toWheelSpeeds((new ChassisSpeeds(xSpeed, 0, rot))));
    }

    public void updateOdometry() {
        // encoders for CANSparkMax don't work in sim mode!
        // double leftDistance =  frontLeftEncoder.getPosition() * kWheelRadius * 2 * Math.PI / kEncoderResolution;
        // double rightDistance = frontRightEncoder.getPosition() * kWheelRadius * 2 * Math.PI /kEncoderResolution;

        // this number is changing..... for PIDs
        // System.out.println("left position: " + frontLeftEncoder.getPosition() + ", distance=" + leftDistance);
        // System.out.println("right position: " + frontRightEncoder.getPosition() + ", distance=" + rightDistance);

        // m_odometry.update(m_gyro.getRotation2d(), leftDistance, rightDistance);
    }

    public void resetOdometry(Pose2d pose) {
        Chassis.fLeft.reset();
        Chassis.fRight.reset();
        Chassis.bLeft.reset();
        Chassis.bRight.reset();
        m_drivetrainSimulator.setPose(pose);
        // m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    // public Pose2d getPose() {
    //     return m_odometry.getPoseMeters();
    // }

     /** Update our simulation. This should be run every robot loop in simulation. */
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
       
        // noPids
        if (!Robot.usePIDs) {
           double lv = -Chassis.fRight.get() * RobotController.getInputVoltage();
           double rv = -Chassis.fLeft.get() * RobotController.getInputVoltage();
        //    System.out.println(String.format("lv: %2.2f, rv: %2.2f", lv, rv));
           m_drivetrainSimulator.setInputs( lv, rv);
        } else {
           double lv = -(Chassis.fRight.getEncoder().getVelocity() / 5700.0)* RobotController.getInputVoltage();
           double rv = -(Chassis.fLeft.getEncoder().getVelocity() / 5700.0)* RobotController.getInputVoltage();
        //    System.out.println(String.format("lv: %2.2f, rv: %2.2f", lv, rv));
           m_drivetrainSimulator.setInputs( lv, rv);
        }
        // System.out.println(String.format("left vel: %2.2f, right vel: %2.2f", Chassis.fLeft.getEncoder().getVelocity(),
        //     Chassis.fRight.getEncoder().getVelocity()));
        
        m_drivetrainSimulator.update(0.02);
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    public void periodic() {
        updateOdometry();
        // if (Robot.usePIDs) {
        //     m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
        // } else {
           m_fieldSim.setRobotPose(m_drivetrainSimulator.getPose());
        // }
      
    }

}
