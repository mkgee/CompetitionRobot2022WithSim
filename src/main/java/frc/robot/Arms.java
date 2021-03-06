package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.helpers.CCSparkMax;
import frc.helpers.OI;
import frc.helpers.PneumaticsSystem;
import frc.parent.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;


public class Arms implements RobotMap {
 
    public static CCSparkMax climber = new CCSparkMax("Climber", "C", RobotMap.CLIMBER, 
        MotorType.kBrushless, IdleMode.kBrake, RobotMap.CLIMBER_LEFT_REVERSE, true);

    public static PneumaticsSystem armSols = new PneumaticsSystem(PneumaticsModuleType.CTREPCM, RobotMap.ARM_SOLENOID_ONE, RobotMap.ARM_SOLENOID_TWO);
    public static double position;

    public static void nothing(){
        climber.setPositionConversionFactor(1 / ARM_ENCODER_HIGH);
        position = climber.getPosition();
    }

    public static void setArms(boolean on){
        armSols.set(on);
    }

    /** 
     * Toggles the climbing arms.
    */
    public static void toggleArms(){
        armSols.toggle();
    }

    /** 
     * Will run the elevator based on what triggers are true or false
     *@param upTrigger what triggers the elevator with positive speed (takes precedence over triggerTwo). Suggest passing in a button or axis input.
     *@param downTrigger what triggers the elevator with negative speed. Suggest passing in a button or axis input.
     *@param hardStop will set speed to 0 (takes precedence over triggers one and two)
     *@param speed the elevator speed
    */
    public static void runElevator(boolean upTrigger, boolean downTrigger, boolean hardStop, double speed){
        if(!calibrated) return;
        // if a trigger is set, set pos to the right encoder to stop the elevator from going
        // otherwise move in the direction of the set pos
        if(hardStop){
            climber.set(0);
            return;
        }
        if(upTrigger){
            if(calibrated) position = climber.getPosition();
            climber.set(speed);
            return;
        }
        if(downTrigger){
            if(climber.getPosition() <= -1 && calibrated) return;
            if(calibrated) position = climber.getPosition();
            climber.set(-speed);
            return;
        }
        double set = 0;
        if(Math.abs(climber.getPosition() - position) > 0.02 && calibrated) set = -Math.abs(climber.getPosition() - position) / (climber.getPosition() - position);
        climber.set(OI.normalize(set, -speed, speed));
    }

    /** 
     * Toggles the climbing arms. Will only trigger again after trigger is false
     *@param trigger what will trigger the toggle. Suggest passing in a button or axis input.
    */
    public static void toggleArms(boolean trigger){
        armSols.triggerSystem(trigger);
    }

    public static final double ARM_ENCODER_HIGH = 165;
    //public static DigitalInput limit = new DigitalInput(RobotMap.ELEVATOR_SWITCH);
    public static boolean calibrated = false;
    public static boolean calibrate(){
        if(calibrated) return true;
        if(!calibrated){
            climber.set(.5);
        }
        if(Robot.limit.get() && !calibrated){
            climber.set(0);
            calibrated = true;
            position = 0;
            climber.reset();
        }
        return calibrated;
    }

    /**
     * 
     * @param position a value from 0-1, with 0 being all the way down, and 1 being all the way up
     */
    public static void setPosition(double position){
        if(calibrated);
        Arms.position = OI.normalize(position, -1, 0);
    }
    
}
