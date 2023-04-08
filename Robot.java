// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxAlternateEncoder;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Define Controller - Controller 2 (CoDriver) Only
  private XboxController m_coController = new XboxController(OIConstants.kCoControllerPort);

  //Define Motor Controllers for Elevator/Arm/Claw
  public static CANSparkMax m_VertLead;
  public static CANSparkMax m_VertFollow;
  public static CANSparkMax m_HoriLead;
  public static CANSparkMax m_HoriFollow;
  public static CANSparkMax m_Flip;
  public static CANSparkMax m_ClawLead;
  public static CANSparkMax m_ClawFollow;

  //Define Pneumatic Controller
  public static Compressor pcmCompressor;
  public static DoubleSolenoid pcmClaw;

  //Swerve Drive Stuff
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Command auto_lift;
  private Command auto_close;
  private Command auto_out;
  private double starttime;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Setup Spark Max Motor Controllers for Elevator/Arm/Claw
    m_VertLead = new CANSparkMax(OIConstants.kVertLeadCAN, MotorType.kBrushless);
    m_VertFollow = new CANSparkMax(OIConstants.kVertFollowCAN, MotorType.kBrushless);
    m_HoriLead = new CANSparkMax(OIConstants.kHoriLeadCAN, MotorType.kBrushed);
    m_HoriFollow = new CANSparkMax(OIConstants.kHoriFollowCAN, MotorType.kBrushed);
    m_Flip = new CANSparkMax(OIConstants.kFlipCAN, MotorType.kBrushless);
    m_ClawLead = new CANSparkMax(OIConstants.kClawLeadCAN, MotorType.kBrushless);
    m_ClawFollow = new CANSparkMax(OIConstants.kClawFollowCAN, MotorType.kBrushless);

    m_VertLead.restoreFactoryDefaults();
    m_VertFollow.restoreFactoryDefaults();
    m_ClawFollow.restoreFactoryDefaults();
    m_ClawLead.restoreFactoryDefaults();

  
   //Setup Follower Motors
   m_VertFollow.follow(m_VertLead, false);
   m_HoriFollow.follow(m_HoriLead);
   m_ClawFollow.follow(m_ClawLead, true);

   m_VertFollow.burnFlash();

    //Setup Camera
    CameraServer.startAutomaticCapture();

    //Setup Double Solenoid
    pcmCompressor = new Compressor(OIConstants.kPneumaticCAN, PneumaticsModuleType.CTREPCM);
    pcmClaw = new DoubleSolenoid(OIConstants.kPneumaticCAN,PneumaticsModuleType.CTREPCM, OIConstants.kSolenoid1, OIConstants.kSolenoid2);
    pcmCompressor.enableDigital();


 
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    starttime = Timer.getFPGATimestamp();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
     m_autonomousCommand = new AutoCommand();
 
     
 
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    

  }

  @Override
  public void teleopInit() {
  
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  
  //Vertical Elevator Code
  if(m_coController.getRightTriggerAxis()>0){
  m_VertLead.set(-(m_coController.getRightTriggerAxis())/2);}
  else if (m_coController.getLeftTriggerAxis()>0){
  m_VertLead.set(-0.02);
  }
  else {
  m_VertLead.set(-0.1);
  }
  //Horizontal Elevator Code
  if (m_coController.getXButton()){
    System.out.print("carriage out");
    m_HoriLead.set(OIConstants.kHoriOutSpeed);
    //m_HoriFollow.set(OIConstants.kHoriOutSpeed);
  }
  else if (m_coController.getYButton()) {
    System.out.print("carriage in");
    m_HoriLead.set(OIConstants.kHoriInSpeed);
    //m_HoriFollow.set(OIConstants.kHoriOutSpeed);
  }
  else {m_HoriLead.set(0);
    //m_HoriFollow.set(0);
  }
  
  //Flip Arm Code
  if (m_coController.getLeftBumper()){
    System.out.print("flip out");
    m_Flip.set(OIConstants.kFlipInSpeed);

  }
  else if (m_coController.getRightBumper()){
    System.out.print("flip in");
    m_Flip.set(OIConstants.kFlipOutSpeed);
  }
  else {m_Flip.set(0.00);}

  //Claw Wheels Code
  if (m_coController.getAButton()){
    System.out.print("claw spin out");
    m_ClawLead.set(OIConstants.kClawOutSpeed);
    //m_ClawFollow.set(OIConstants.kClawOutSpeed);

  }
  else if (m_coController.getBButton()) {
    System.out.print("claw spin out in");
    m_ClawLead.set(OIConstants.kClawInSpeed);
    //m_ClawFollow.set(OIConstants.kClawInSpeed);
  }
  else {
  System.out.print("claw stop");
  m_ClawLead.set(0);
  m_ClawFollow.set(0);
  }

  //Claw Pneumatic Code
  if (m_coController.getPOV() == 90){
    System.out.print("claw open");
    pcmClaw.set(Value.kForward);
  }
  if (m_coController.getPOV() == 270) {
    System.out.print("claw close");
    pcmClaw.set(Value.kReverse);
  }
  if(m_coController.getBackButtonPressed()){
    pcmClaw.set(Value.kReverse);
  }
  if(m_coController.getStartButtonPressed()){
    pcmClaw.set(Value.kForward);
  }
  

  }

  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
