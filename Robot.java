package frc.robot;

//Added imports just now
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import frc.robot.commands.MainAutoCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// REVLib 2026: ResetMode and PersistMode are now top-level com.revrobotics classes
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Robot extends TimedRobot {

  private final double kPShooter = 0.00015;
  private final double kIShooter = 0.0;
  private final double kDShooter = 0.0;
  private final double kVShooter = 0.0023;
  private final double kShooterAllowedClosedLoopError = 0.0;
  private final double kIZoneShooter = 0.0;
  private final Timer m_buttonTimer = new Timer();
  private final Timer m_autoShootTimer = new Timer();
  private boolean lastTriggerPressed = false;


  
     

 
  
  // Auto setup
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();



  // =========================================================
  // HARDWARE DECLARATIONS
  // Drive Train: Left side (IDs 1 & 2), Right side (IDs 3 & 4)
  // =========================================================
  private final SparkMax m_leftFront = new SparkMax(1,                                                                                                            MotorType.kBrushed);
  private final SparkMax m_leftBack = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightFront = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_rightBack = new SparkMax(4, MotorType.kBrushed);

  // Shooter: Back-to-back on one shaft (IDs 5 & 6)
  private final SparkMax m_shooterLeader = new SparkMax(5, MotorType.kBrushless);
  //private final Encoder m_shootEncoder = (Encoder) m_shooterLeader.getEncoder();
  private final SparkMax m_shooterFollower = new SparkMax(6, MotorType.kBrushless);

  // Arm (ID 7)
  // private final SparkMax m_arm = new SparkMax(7, MotorType.kBrushed);

  // DifferentialDrive takes LEFT leader and RIGHT leader.
  // Right side inversion is handled in SparkMax config below, NOT here.
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftFront, m_rightFront);

  // Logitech Xbox controller on USB port 0
  private final XboxController m_controller = new XboxController(1);
  private final XboxController m_operator = new XboxController(0);

  // Deadzone threshold — eliminates idle joystick drift
  private static final double DEADZONE = 0.15;

  // =========================================================
  // SIMULATION OBJECTS
  // =========================================================
  private DifferentialDrivetrainSim m_driveSim;
  private final Field2d m_field = new Field2d(); // AdvantageScope reads this

  // =========================================================


  /* 
  private void shake(double speed, double time, double currentTime)
  {
    if (currentTime < time/2) 
    {
      m_robotDrive.arcadeDrive(speed, 0);
    }
    else if (currentTime < time)
    {
      m_robotDrive.arcadeDrive(-speed, 0);
    }
  }
  */

  public static final double MOVE_THRESHOLD = 0.1;
  public static final double MOVE_CURVE = 1.5;
  public static final double TURN_THRESHOLD = 0.1;
  public static final double TURN_CURVE = 1.5;

  @Override
  public void robotInit() {

    SmartDashboard.putData("Field", m_field);

    // Set default auto to do nothing, and adding the option for other autos
      m_chooser.setDefaultOption("Move back and shoot", "MainAuto");
      m_chooser.addOption("No auto selected", null);
   // m_chooser.setDefaultOption("Move back and shoot",  getMainAutoCommand()); 

    
    SmartDashboard.putData("Auto Chooser", m_chooser);
    SmartDashboard.putData("Drivebase", m_robotDrive);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());

    // Publish field to NetworkTables so AdvantageScope can find it
    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber("Constants/kIntakeSpeedInnerWheels", -0.6);
    SmartDashboard.putNumber("Constants/kIntakeSpeedOuterWheels", -0.45);

    // ---------------------------------------------------------
    // 1. LEFT FRONT (ID 1) — Drive leader, no inversion
    // ---------------------------------------------------------
    SparkMaxConfig leftFrontConfig = new SparkMaxConfig(); // container for motor settings before sending it to motor
    leftFrontConfig
        .idleMode(IdleMode.kBrake) // stops quickly when power stops
        .smartCurrentLimit(40);
    m_leftFront.configure(
        leftFrontConfig,
        ResetMode.kResetSafeParameters, // resets important settings to safe defaults before applying new ones
        PersistMode.kPersistParameters); // saves settings

    // ---------------------------------------------------------
    // 2. LEFT BACK (ID 2) — Follows left front, same direction
    // ---------------------------------------------------------
    SparkMaxConfig leftBackConfig = new SparkMaxConfig();
    leftBackConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .follow(m_leftFront, false); 
    m_leftBack.configure(
        leftBackConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // ---------------------------------------------------------
    // 3. RIGHT FRONT (ID 3) — Drive leader, INVERTED
    // Physically spins opposite for correct forward drive
    // ---------------------------------------------------------
    SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
    rightFrontConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(true);
    m_rightFront.configure(
        rightFrontConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // ---------------------------------------------------------
    // 4. RIGHT BACK (ID 4) — Follows right front
    // Leader is already inverted, so false here
    // ---------------------------------------------------------
    SparkMaxConfig rightBackConfig = new SparkMaxConfig();
    rightBackConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .follow(m_rightFront, false);
    m_rightBack.configure(
        rightBackConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // ---------------------------------------------------------
    // 5. SHOOTER LEADER (ID 5) — No inversion, coast mode
    // ---------------------------------------------------------
    SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
    shooterLeaderConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast) // moves freely even when power is off
        .smartCurrentLimit(40);
    shooterLeaderConfig.closedLoop 
        .pid(kPShooter, kIShooter, kDShooter)
        .iZone(kIZoneShooter)
        .allowedClosedLoopError(kShooterAllowedClosedLoopError, ClosedLoopSlot.kSlot0).feedForward.kV(kVShooter);
    m_shooterLeader.configure(
        shooterLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // ---------------------------------------------------------
    // 6. SHOOTER FOLLOWER (ID 6) — Inverted relative to leader
    // Motors face each other on same shaft — must spin opposite
    // ---------------------------------------------------------
    SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
    shooterFollowerConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40);
    // .follow(m_shooterLeader, true); // true = inverted relative to leader
    m_shooterFollower.configure(
        shooterFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // ---------------------------------------------------------
    // 7. ARM (ID 7) — Standalone, brake mode holds position
    // ---------------------------------------------------------
    /*
     * SparkMaxConfig armConfig = new SparkMaxConfig();
     * armConfig
     * .idleMode(IdleMode.kBrake)
     * .smartCurrentLimit(30);
     * m_arm.configure(
     * armConfig,
     * ResetMode.kResetSafeParameters,
     * PersistMode.kPersistParameters);
     */

    // Safety watchdog — feed() in teleopPeriodic keeps this alive
    m_robotDrive.setSafetyEnabled(true);

    System.out.println("[Robot] robotInit complete — all 7 Spark Max motors configured.");
  }

  // =========================================================
  // SIMULATION INIT — only runs in simulation, never on real robot
  // =========================================================
  @Override
  public void simulationInit() {
    m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide,
        KitbotGearing.k10p71,
        KitbotWheelSize.kSixInch,
        null);

    // Starting position of the simulated robot
    Pose2d startingPose = new Pose2d(
        2.0,                         // X = 2 meters
        3.0,                         // Y = 3 meters
        Rotation2d.fromDegrees(0)    // Facing 0 degrees
    );

    m_driveSim.setPose(startingPose);
    m_field.setRobotPose(startingPose);

    System.out.println("[SIM] Differential drivetrain simulation initialized!");
  }

  // =========================================================
  // SIMULATION PERIODIC — physics update + AdvantageScope feed
  // =========================================================
  @Override
  public void simulationPeriodic() {
   /*  if (isDisabled()) {
      // Zero out inputs when disabled to prevent phantom drift
      m_driveSim.setInputs(0.0, 0.0);
    } else {
      // Right side is inverted in hardware config (inverted(true) above).
      // DifferentialDrivetrainSim expects raw undirected voltage on both sides,
      // so we negate the right side here to cancel out the hardware inversion.
      // Without this negation, left=-0.158 and right=+0.158 causes endless spinning.
      double leftInput = applyDeadzone(m_leftFront.get()) * 12.0;
      double rightInput = -applyDeadzone(m_rightFront.get()) * 12.0; // negate to un-invert
      m_driveSim.setInputs(leftInput, rightInput);
*/
      //Added code
      //-------------------------------------------------
 // Get the motor outputs from our drive code.
  double leftOutput = m_leftFront.get();
  double rightOutput = m_rightFront.get();

  // Convert motor percentage (-1 to +1)
  // into voltage (-12 to +12).
  double leftVoltage = leftOutput * 12.0;
  double rightVoltage = rightOutput * 12.0;

  // Our physical right motor is inverted.
  // The drivetrain simulator expects both sides
  // to represent physical wheel direction.
  rightVoltage = -rightVoltage;

  // If robot is disabled, don't let it move.
  if (isDisabled()) {
    leftVoltage = 0.0;
    rightVoltage = 0.0;
  }

  // Send motor voltages into the physics simulation.
  m_driveSim.setInputs(leftVoltage, rightVoltage);

  // Advance simulation by 20 milliseconds.
  m_driveSim.update(0.020);

  // Get simulated robot position.
  Pose2d pose = m_driveSim.getPose();

  // Put robot position onto the simulated field.
  m_field.setRobotPose(pose);

  // Useful information in SmartDashboard.
  SmartDashboard.putNumber("Sim/X", pose.getX());
  SmartDashboard.putNumber("Sim/Y", pose.getY());
  SmartDashboard.putNumber(
      "Sim/Heading",
      pose.getRotation().getDegrees());

  SmartDashboard.putNumber(
      "Sim/LeftVelocity",
      m_driveSim.getLeftVelocityMetersPerSecond());

  SmartDashboard.putNumber(
      "Sim/RightVelocity",
      m_driveSim.getRightVelocityMetersPerSecond());

      //----------------------------------------------
    

    // getPose() is the correct WPILib 2026 call — replaces old getXPositionMeters()
    // etc.
    Pose2d simPose = m_driveSim.getPose();

    // Push pose to Field2d — AdvantageScope reads this from NetworkTables
    m_field.setRobotPose(simPose);

    // Telemetry — visible in AdvantageScope left panel under Sim/
    SmartDashboard.putNumber("Sim/X", simPose.getX());
    SmartDashboard.putNumber("Sim/Y", simPose.getY()); 
    SmartDashboard.putNumber("Sim/HeadingDeg", simPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Sim/LeftVelocity", m_driveSim.getLeftVelocityMetersPerSecond());
    SmartDashboard.putNumber("Sim/RightVelocity", m_driveSim.getRightVelocityMetersPerSecond());
  }

  double m_autoStartTime = 0;

  @Override
  public void autonomousInit()
  {
    m_autoStartTime = Timer.getFPGATimestamp();
    
    m_autoSelected = m_chooser.getSelected();
    /* 
   
      CommandScheduler.getInstance().schedule(m_autoSelected);
    }
    */
  }
  

  @Override
  public void autonomousPeriodic()
  { 
    m_autoShootTimer.start();
    double currentTimeDiff = Timer.getFPGATimestamp() - m_autoStartTime;
    double lastTime;

    SmartDashboard.putNumber("autoStartTime", m_autoStartTime);
    SmartDashboard.putNumber("autoCurrentDiff", currentTimeDiff);

    if (currentTimeDiff < 1.0) {
        m_robotDrive.arcadeDrive(-0.5, 0);
    }
    else if (currentTimeDiff < 4.0) {
          m_autoShootTimer.restart();
          m_robotDrive.arcadeDrive(0, 0);
          m_shooterLeader.set(-0.9);
          if (m_autoShootTimer.hasElapsed(1.5))
          {
            m_shooterFollower.set(0.9);
          }
          
    }
     
    else if (currentTimeDiff < 4.2)
    {
      m_robotDrive.arcadeDrive(0.80, 0);
    }
      
    else if (currentTimeDiff < 4.4) 
    {
      m_robotDrive.arcadeDrive(-0.80, 0);
      //shake(0.8, 0.4, currentTimeDiff);
    }
    else if (currentTimeDiff < 10.0) {
          m_autoShootTimer.restart();
          m_robotDrive.arcadeDrive(0, 0);
          m_shooterLeader.set(-0.9);
          if (m_autoShootTimer.hasElapsed(1.5))
          {
            m_shooterFollower.set(0.9);
          }
    }
    else if (currentTimeDiff < 10.2)
    {
      m_robotDrive.arcadeDrive(0.80, 0);
    }
    else if (currentTimeDiff < 10.4) 
    {
      m_robotDrive.arcadeDrive(-0.80, 0);
      //shake(0.8, 0.4, currentTimeDiff);
    }
    else 
    {
      m_autoShootTimer.restart();
      m_robotDrive.arcadeDrive(0, 0);
      m_shooterLeader.set(-0.9);
      if (m_autoShootTimer.hasElapsed(1.5))
      {
        m_shooterFollower.set(0.9);
      }
    }
 
    m_robotDrive.feed();
    /* 
    if(m_autoSelected == null)
    {
      System.out.println("Null Auto");
    }
    else if (m_autoSelected.equals("MainAuto")) 
    {
      System.out.println("Main Auto");
      moveBackAndShoot();
    }
*/
  
  }

   boolean firstShoot = true;
   double shootStartTime;
  double shootTime;

  @Override
  public void teleopInit()
  {
    firstShoot = true;
    shootStartTime = Timer.getFPGATimestamp();
  }
  
  

  // =========================================================
  // TELEOP PERIODIC
  // =========================================================
  @Override
  public void teleopPeriodic() {

    // ---------------------------------------------------------
    // DRIVE — left stick = forward/back, right stick = turn
    // ---------------------------------------------------------
    double move = applyDeadzone(-m_controller.getLeftY()); // Negate: push forward = positive

    //double move = (m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis());

    double turn = applyDeadzone(m_controller.getRightX());

    SmartDashboard.putNumber("Values/move", move);
    SmartDashboard.putNumber("Values/turn", turn);

    m_robotDrive.arcadeDrive(moveCurve(move, 1), turnCurve(turn, 0.75));
    m_robotDrive.feed(); // Keeps safety watchdog alive — critical for sim Applied Output

    // ---------------------------------------------------------
    // SHOOTER — A = shoot forward, B = intake reverse
    // Only leader needs .set(); follower handled by REV firmware
    // ---------------------------------------------------------
 
    double intakeSpeedInnerWheels = SmartDashboard.getNumber("Constants/kIntakeSpeedInnerWheels", -1.0);
    double intakeSpeedOuterWheels = SmartDashboard.getNumber("Constants/intakeSpeedOuterWheels", -1.0);
    
    boolean triggerPressed = m_operator.getRightTriggerAxis() > 0;
    
    // Shoot
    if (triggerPressed) 
    {
      if(triggerPressed)
      {
      m_shooterLeader.set(-4.0);
      if(!lastTriggerPressed)
      {
        m_buttonTimer.restart();
      }
      if(m_buttonTimer.hasElapsed(1.2))
      { 
        m_shooterFollower.set(4.0);
      }
      }
    } 
    // Intake
    else if (m_operator.getLeftTriggerAxis() > 0) 
    {
      m_shooterLeader.set(-0.30);
      m_shooterFollower.set(-1.0);
    } 
    // Outtake --> spits out 
    else if (m_operator.getAButton())
    {
      m_shooterLeader.set(1);
      m_shooterFollower.set(1);
    }
    // Intake/shoot immediately 
    else if (m_operator.getXButton())
    {
      m_shooterLeader.set(-1.0);

         m_buttonTimer.restart();
        
         double timeSincePress = m_buttonTimer.get();

         if(m_buttonTimer.isRunning() && timeSincePress < 3.0) 
         {
          m_shooterFollower.set(1);
         }
    }
    // Into basket
    else if (m_operator.getBButton())
    {
      m_shooterLeader.set(1);
      m_shooterFollower.set(-1);
    }
    else if (m_operator.getYButton())
    {
        m_buttonTimer.stop();
        m_buttonTimer.reset();
    }
    else if(m_operator.getRightBumperButton())
    {
      if(m_operator.getRightBumperButton())
      {
        m_shooterLeader.set(-6.0);
      if(!lastTriggerPressed)
      {
        m_buttonTimer.restart();
      }
      if(m_buttonTimer.hasElapsed(1.5))
      {
        m_shooterFollower.set(1.0);
      }
      }
    }
    else 
    {
      m_shooterLeader.set(0.0);
      m_shooterFollower.set(0.0);
      firstShoot = true;
    }

    lastTriggerPressed = triggerPressed;

    // ---------------------------------------------------------
    // ARM — Right bumper = up, Left bumper = down
    // WPILib 2026: getRightBumperButton() replaces deprecated getRightBumper()
    // ---------------------------------------------------------
    /*
     * if (m_controller.getRightBumperButton()) {
     * m_arm.set(0.25);
     * } else if (m_controller.getLeftBumperButton()) {
     * m_arm.set(-0.25);
     * } else {
     * m_arm.set(0.0);
     * }
     */

    // ---------------------------------------------------------
    // SMARTDASHBOARD — verify all subsystems in sim or on robot
    // ---------------------------------------------------------
    SmartDashboard.putNumber("Drive/Move Input", move);
    SmartDashboard.putNumber("Drive/Turn Input", turn);
    SmartDashboard.putNumber("Drive/LeftPower", m_leftFront.get());
    SmartDashboard.putNumber("Drive/RightPower", m_rightFront.get());
    SmartDashboard.putNumber("Shooter/Power", m_shooterLeader.get());
    // SmartDashboard.putNumber("Arm/Power", m_arm.get());
  }

  // =========================================================
  // DISABLED PERIODIC — feed watchdog to suppress console warnings
  // =========================================================
  @Override
  public void disabledPeriodic() {
    m_robotDrive.feed();
  }

  // =========================================================
  // HELPER: Deadzone — returns exactly 0.0 within threshold
  // =========================================================
  private double applyDeadzone(double value) {
    if (Math.abs(value) < DEADZONE) {
      return 0.0;
    }
    return value;
  }

  private void delayedShooting(double leaderSpeed, double followerSpeed, double triggerValue, double delay)
  {
    m_shooterLeader.set(-leaderSpeed * triggerValue);
    if(!firstShoot && shootTime > delay)
      {
        m_shooterFollower.set(followerSpeed * triggerValue);
      }
      else if (firstShoot)
      {
        shootStartTime = Timer.getFPGATimestamp();
        shootTime = Timer.getFPGATimestamp() - shootStartTime;
        m_shooterFollower.set(0);
        firstShoot = false;
      }
      else
      {
        shootTime = Timer.getFPGATimestamp() - shootStartTime;
        m_shooterFollower.set(0);
      }
  }

  public void moveBackAndShoot()
  {
    double currentTimeDiff = Timer.getFPGATimestamp() - m_autoStartTime;

    SmartDashboard.putNumber("autoStartTime", m_autoStartTime);
    SmartDashboard.putNumber("autoCurrentDiff", currentTimeDiff);
    delayedShooting(0.8, 0.8, 1.0,  3.0);

    if (currentTimeDiff < 0.75) {
      m_robotDrive.arcadeDrive(-0.5, 0);
      m_shooterFollower.set(0);
      m_shooterLeader.set(0);
    }
    else if (currentTimeDiff < 2.0) 
    {
      m_shooterLeader.getClosedLoopController().setReference(-2150, SparkMax.ControlType.kVelocity);
      m_robotDrive.arcadeDrive(0, 0);
    }
    else if(currentTimeDiff < 8.0)
    {
      m_shooterFollower.set(0.50);
    }
    else if (currentTimeDiff < 8.125) 
    {
      m_shooterFollower.set(0);
      m_robotDrive.arcadeDrive(0.7, 0);
    }
    else if (currentTimeDiff < 8.3 )
    {
      m_robotDrive.arcadeDrive(-0.7, 0);
    }
    else if (currentTimeDiff < 13) 
    {
      m_shooterFollower.set(0.50);
      m_robotDrive.arcadeDrive(0, 0);

    }
    else if(currentTimeDiff < 13.5)
    {      
      m_shooterLeader.getClosedLoopController().setReference(0, SparkMax.ControlType.kVelocity);
      m_shooterFollower.set(0);
      m_robotDrive.arcadeDrive(0.4, 0);
    }
    else
    {
      m_robotDrive.arcadeDrive(0, 0);
      m_shooterLeader.getClosedLoopController().setReference(0, SparkMax.ControlType.kVelocity);
      m_shooterFollower.set(0);
    }
 
    m_robotDrive.feed();
  }


  private double moveCurve(double joystick, double speed) // returns a curve adjusted movement for the joystick, better for fine position control
  {
    if (joystick > 0) 
    {
      return (1 - MOVE_THRESHOLD) * Math.pow(joystick, MOVE_CURVE) + MOVE_THRESHOLD;
    }
    else if (joystick < 0) 
    {
      return (1 - MOVE_THRESHOLD) * -Math.pow(Math.abs(joystick), MOVE_CURVE) - MOVE_THRESHOLD;
    }
    return 0;
  }

  private double turnCurve(double joystick, double speed) // returns a curve adjusted turning for the joystick, better for fine position control
  {
    if (joystick > 0) 
    {
      return (1 - TURN_THRESHOLD) * Math.pow(joystick, TURN_CURVE) + TURN_THRESHOLD;
    }
    else if (joystick < 0) 
    {
      return (1 - TURN_THRESHOLD) * -Math.pow(Math.abs(joystick), TURN_CURVE) - TURN_THRESHOLD;
    }
    return 0;
  }
    
}
