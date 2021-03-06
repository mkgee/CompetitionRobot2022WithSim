package frc.robot;

import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.helpers.PneumaticsSystem;
import frc.parent.*;
//import frc.raspi.Vision;

import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Chassis{

    public static void nothing(){};

    //Talon objects for the wheels
    //These control the main 4 motors on the robot
    public static CCSparkMax fLeft = new CCSparkMax("Front Left", "FL", RobotMap.FORWARD_LEFT, 
        MotorType.kBrushless, IdleMode.kBrake, RobotMap.FORWARD_LEFT_REVERSE, true);
    public static SparkMaxPIDController fLeftPidController = fLeft.getPIDController();

    public static CCSparkMax fRight = new CCSparkMax("Front Right", "FR", RobotMap.FORWARD_RIGHT, 
        MotorType.kBrushless, IdleMode.kBrake, RobotMap.FORWARD_RIGHT_REVERSE, true);
    public static SparkMaxPIDController fRightPidController = fRight.getPIDController();

    public static CCSparkMax bLeft = new CCSparkMax("Back Left", "BL",RobotMap.BACK_LEFT, 
        MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE, true);
    public static SparkMaxPIDController bLeftPidController = bLeft.getPIDController();

    public static CCSparkMax bRight = new CCSparkMax("Back Right", "BR", RobotMap.BACK_RIGHT, 
        MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_RIGHT_REVERSE, true);
    public static SparkMaxPIDController bRightPidController = bRight.getPIDController();

    public static PneumaticsSystem shift = new PneumaticsSystem(PneumaticsModuleType.CTREPCM, RobotMap.SHIFT_SOLENOID_ONE, RobotMap.SHIFT_SOLENOID_TWO);

    //AHRS gyro measures the angle of the bot
    public static AHRS gyro = new AHRS(SPI.Port.kMXP);

    public static int autoStep = 0;

    // PID fields
    private static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    static {
        // PID coefficients
        kP = 6e-5; //8.5; // 6e-5;
        kI = 0;;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 5700; //1;
        kMinOutput = -5700; //-1;
        maxRPM = 5700;

        Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> {
             pc.setP(kP);
             pc.setI(kI);
             pc.setD(kD);
             pc.setIZone(kIz);
             pc.setFF(kFF);
             pc.setOutputRange(kMinOutput, kMaxOutput);
         });

         SmartDashboard.putNumber("P Gain", kP);
         SmartDashboard.putNumber("I Gain", kI);
         SmartDashboard.putNumber("D Gain", kD);
         SmartDashboard.putNumber("I Zone", kIz);
         SmartDashboard.putNumber("Feed Forward", kFF);
         SmartDashboard.putNumber("Max Output", kMaxOutput);
         SmartDashboard.putNumber("Min Output", kMinOutput);
    }

    //To be used in TeleOP
    //Takes in two axises, most likely the controller axises
    //Optimized for a west coast or standard chassis
    //DO NOT USE THIS FOR SWERV DRIVE 
    public static void axisDrive(double yAxis, double xAxis, double max){
        fLeft.set(-OI.normalize((yAxis - xAxis), -max, max));
        fRight.set(-OI.normalize((yAxis + xAxis), -max, max));
        bLeft.set(-OI.normalize((yAxis - xAxis), -max, max));
        bRight.set(-OI.normalize((yAxis + xAxis), -max, max));
    }

    public static void pidDrive(double yAxis, double xAxis, double max) {

        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double maxOutput = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { 
            Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> 
                pc.setP(p)
            );
            kP = p; 
        }
        if((i != kI)) { 
            Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> 
                pc.setI(i)
            );
            kI = i; 
        }
        if((d != kD)) { 
            Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> 
                pc.setD(d)
            );
            kD = d; 
        }
        if((iz != kIz)) { 
            Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> 
                pc.setIZone(iz)
            );
            kIz = iz; 
        }
        if((ff != kFF)) { 
            Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> 
                pc.setFF(ff)
            );
            kFF = ff; 
        }
        if((maxOutput != kMaxOutput) || (min != kMinOutput)) { 
            Stream.of(Chassis.fLeftPidController, Chassis.fRightPidController, Chassis.bLeftPidController, Chassis.bRightPidController).forEach(pc -> 
                pc.setOutputRange(min, maxOutput)
            );
            kMinOutput = min; 
            kMaxOutput = maxOutput; 
        }
        fLeftPidController.setReference(-OI.normalize((yAxis - xAxis), -max, max) * maxRPM, ControlType.kVelocity);
        fRightPidController.setReference(-OI.normalize((yAxis + xAxis), -max, max)* maxRPM, ControlType.kVelocity);
        bLeftPidController.setReference(-OI.normalize((yAxis - xAxis), -max, max)* maxRPM, ControlType.kVelocity);
        bRightPidController.setReference(-OI.normalize((yAxis + xAxis), -max, max)* maxRPM, ControlType.kVelocity);
    }


    //To be used on Auto/PIDs
    //Simply sets the motor controllers to a certain percent output
    public static void driveSpd(double lSpeed, double rSpeed){
        fLeft.set(OI.normalize(lSpeed, -1.0, 1.0));
        fRight.set(OI.normalize(rSpeed, -1.0, 1.0));
        bLeft.set(OI.normalize(lSpeed, -1.0, 1.0));
        bRight.set(OI.normalize(rSpeed, -1.0, 1.0));
    }

    public static void setFactor(double factor){
        //0.048 slow, 0.109 fast
        fLeft.setPositionConversionFactor(factor);
        fRight.setPositionConversionFactor(factor);
        bLeft.setPositionConversionFactor(factor);
        bRight.setPositionConversionFactor(factor);

    }

   
    //Sets the gyro and encoders to zero
    public static void reset(){
        gyro.reset();
        fLeft.reset();
        fRight.reset();
        bLeft.reset();
        bRight.reset();   
        
   }

    public static double getLDist(){
        double dist = (fLeft.getPosition() + bLeft.getPosition())/2;
        return dist;
    }

    public static double getRDist(){
        double dist = (fRight.getPosition() + bRight.getPosition())/2;
        return dist;
    }

    public static double getAngle(){
        return gyro.getAngle();
    }

    /*
        "Whosever holds these loops, if he be worthy, shall posses the power of AJ"
    */
    public static void setFastMode(boolean on){
        shift.set(on);
    }

    /** 
     * Toggles fast mode.
    */
    public static void toggleFastMode(){
        shift.toggle();
    }

    /** 
     * Toggles fast mode. Will only trigger again after trigger is false
     *@param trigger what will trigger the toggle. Suggest passing in a button or axis input.
    */
    public static void toggleFastMode(boolean trigger){
        shift.triggerSystem(trigger);
    }

    //Drives the robot to a certain distance
    //Kinda complex -> DO NOT TOUCH
    public static void driveDist(double goal, double aPer, double kp, double max, boolean debug){
        setFactor(0.048);
        double aError = goal*aPer;

        double lPos = getLDist();
        double lError = goal-lPos;
        double lSpd = 0;

        double rPos = getRDist();
        double rError = goal-rPos;
        double rSpd = 0; 

        while(true){
            lPos = getLDist();
            lError = goal-lPos;
            lSpd = lError*kp;
            lSpd = OI.normalize(lSpd, -max, max);

            rPos = getRDist();
            rError = goal-rPos;
            rSpd = rError*kp;
            rSpd = OI.normalize(rSpd, -max, max);

            driveSpd(lSpd, rSpd);

            if(debug){
                System.out.println("Left - Left Speed: " + lSpd + 
                                        " Left Error: " + lError + 
                                        " Left Position: " + lPos);
                System.out.println("Right - Right Speed: " + rSpd + 
                                        " Right Error: " + rError + 
                                        " Right Position" + rPos);
                Timer.delay(0.5);
            }

            if(lError <= aError && rError <= aError){
                driveSpd(0.0, 0.0);
                System.out.println("YOINK, ya made it");
                break; 
            }
        }
    }

    //Turns the robot to a certain angle, a positive angle will turn right
    //Kinda complex -> DO NOT TOUCH
    public static void turnToAngle(double goal, double aPer, double kp, double max, boolean debug){

        double aError = goal*aPer;

        double angl = gyro.getAngle();
        double error = goal-angl;
        double input = 0;

        while(true){
            angl = gyro.getAngle();
            error = goal-angl;
            input = error*kp;
            input = OI.normalize(input, -max, max);

            driveSpd(input, -input);

            if(debug){
                System.out.println("Input: " + input);
                System.out.println("Error: " + error);
                System.out.println("Angle: " + angl);
                Timer.delay(0.5);
            }

            if(error <= aError){
                driveSpd(0.0, 0.0);
                System.out.println("YOINK, ya made it");
                break; 
            }
        }
    }

    public static boolean turnToAnglePeriodic(double goal, double kp, double max, double aError, int step){
        if(step != autoStep) return true;
        double angl = gyro.getAngle();
        double error = goal-angl;
        double input = error*kp;
        input = OI.normalize(input, -max, max);

        driveSpd(input, -input);

        if(error <= aError){
            driveSpd(0.0, 0.0);
            System.out.println("YOINK, ya made it");
            autoStep++;
            return true;
        }
        return false;
    }

    public static boolean driveDistPeriodic(double goal, double kp, double max, double aError, int step){
        if(step != autoStep) return true;
        setFactor(0.048);
        double lPos = getLDist();
        double lError = goal-lPos;
        double lSpd = lError*kp;
        lSpd = OI.normalize(lSpd, -max, max);

        double rPos = getRDist();
        double rError = goal-rPos;
        double rSpd = rError*kp;
        rSpd = OI.normalize(rSpd, -max, max);

        driveSpd(lSpd, rSpd);

        if(lError <= aError && rError <= aError){
            driveSpd(0.0, 0.0);
            System.out.println("YOINK, ya made it");
            autoStep++;
            return true;
        }
        return false;
    }
    
}