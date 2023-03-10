/*
  2022 everybot code
  written by carson graf 
  don't email me, @ me on discord
*/

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

/*import java.sql.Time;
import java.sql.Timestamp;
import java.text.BreakIterator;
import java.util.concurrent.TimeUnit;*/

//import com.ctre.phoenix.time.StopWatch;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.*;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;

// import com.revrobotics.RelativeEncoder;

// import com.revrobotics.SparkMaxPIDController; 
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.ctre.phoenix.signals.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //Definitions for the hardware. Change this if you change what stuff you have plugged in
  PWMVictorSPX driveLeftA = new PWMVictorSPX(1);
  PWMVictorSPX driveLeftB = new PWMVictorSPX(2);
  PWMVictorSPX driveRightA = new PWMVictorSPX(3);
  PWMVictorSPX driveRightB = new PWMVictorSPX(4);
  CANSparkMax arm = new CANSparkMax(11, MotorType.kBrushless);
  PWMVictorSPX intake = new PWMVictorSPX(0);

  Joystick driverController = new Joystick(0);
  Joystick armController = new Joystick(1);

  //CANSparkMax.getEncoder();
  //RelativeEncoder
  //Constants for controlling the arm. consider tuning these for your particular robot
  final double armHoldUp = 0.05;
  final double armHoldDown = 0.10;
  final double armTravel1 = 0.5;

  final double armTimeUp = 0.5;
  final double armTimeDown = 0.5;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0; 

  double autoStart = 0;
  boolean goForAuto = true;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Configure motors to turn correct direction. You may have to invert some of your motors
    driveLeftA.setInverted(true);
    // driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    // driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    // driveRightA.burnFlash();
    driveRightB.setInverted(false);
    // driveRightB.burnFlash();
    
    arm.setInverted(false);
    arm.setIdleMode(IdleMode.kBrake);
    ((CANSparkMax) arm).burnFlash();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //arm control code. same as in teleop
    if(armUp){
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        arm.set(armTravel1);
        //arm.set(VictorSPXControlMode.PercentOutput, armTravel);
      }
      else{
        arm.set(armHoldUp);
        //arm.set(VictorSPXControlMode.PercentOutput, armHoldUp);
      }
    }
    /*else{
      if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        arm.set(-armTravel);
        //arm.set(VictorSPXControlMode.PercentOutput, -armTravel);
      }
      else{
        arm.set(-armHoldUp);
        //arm.set(VictorSPXControlMode.PercentOutput, -armHoldUp);
      }
    }*/
    
    //get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){
      //series of timed events making up the flow of auto
      //if(autoTimeElapsed < 3){
        //spit out the ball for three seconds
        //intake.set(-1);
      }if(autoTimeElapsed < 3){
        //stop spitting out the ball and drive backwards *slowly* for three seconds
        intake.set(-0.3);
        driveLeftA.set(-0.3);
        driveLeftB.set(-0.3);
        driveRightA.set(-0.3);
        driveRightB.set(-0.3);
      }else{
        //do nothing for the rest of auto
        intake.set(0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(4);
    
    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);

    // new arm controls
    double initialTime = Timer.getFPGATimestamp();
    double finalTime = Timer.getFPGATimestamp();
    double timeElapsed = finalTime - initialTime;

    double upAndDown = -armController.getRawAxis(1);

    double armPower = upAndDown - 0.5;  

    // StopWatch watch = new StopWatch();
    
    // watch.start();
    
    arm.set(armPower/4);
    final double armTravel = armPower * timeElapsed;

    if (armTravel > 0.2) {
      arm.set(0);
    }
    
    //Intake controls
    if(armController.getRawButton(0)){
      intake.set(1);
    }
    else if(armController.getRawButton(1)){
      intake.set(-1);
    }
    else{
      intake.set(0);
    }


    //Arm Controls
    
    // if(armUp){
      // if(Timer.getFPGATimestamp() - lastBurstTime < armTimeUp){
        // arm.set(armTravel);
        //arm.set(VictorSPXControlMode.PercentOutput, armTravel);
      // }
      // else{
        // arm.set(armHoldUp);
        //arm.set(VictorSPXControlMode.PercentOutput, armHoldUp);
      // }
    // }
    // else{
      // if(Timer.getFPGATimestamp() - lastBurstTime < armTimeDown){
        // arm.set(-armTravel);
        //arm.set(VictorSPXControlMode.PercentOutput, -armTravel);
      }
      // else{
        // arm.set(-armHoldUp);
        //arm.set(VictorSPXControlMode.PercentOutput, -armHoldDown);
      // }
    // }
  
    // if(armController.getRawButtonPressed(2) && !armUp){
      // lastBurstTime = Timer.getFPGATimestamp();
      // armUp = true;
    // }
    // else if(armController.getRawButtonPressed(3) && armUp){
      // lastBurstTime = Timer.getFPGATimestamp();
      // armUp = false;
    // }  

  // }

  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    arm.set(0);
    //arm.set(VictorSPXControlMode.PercentOutput, 0);
    intake.set(0);
  }
}