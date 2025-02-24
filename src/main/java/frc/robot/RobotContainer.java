
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.OrientWithLL;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.AlgaeHoldCommands.AlgaeGrab;
import frc.robot.commands.AlgaeHoldCommands.AlgaeHighPickup;
import frc.robot.commands.AlgaeHoldCommands.AlgaeLowPickup;
import frc.robot.commands.AlgaeHoldCommands.AlgaeRelease;
import frc.robot.commands.AlgaePivotCommands.ManualAlgaePivot;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
//mport frc.robot.commands.AutoCommands.DriveWithPath;
import frc.robot.commands.AutoCommands.TurnOnly;
import frc.robot.commands.AutoCommands.Center.CtrScore1;
import frc.robot.commands.AutoCommands.Center.FullRunLeftCtr;
import frc.robot.commands.AutoCommands.Right.FullRunRight;
import frc.robot.commands.AutoCommands.Right.Leg1Right;
import frc.robot.commands.AutoCommands.Right.Leg2Right;
import frc.robot.commands.AutoCommands.Right.Leg3Right;
import frc.robot.commands.AutoCommands.Right.Legs1and2Right;
import frc.robot.commands.AutoCommands.Right.FullRunRight;
import frc.robot.commands.CoralHoldCommands.CoralGrabWithCounter;
import frc.robot.commands.CoralHoldCommands.CoralGrab;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.ManualCoralPivot;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ManualUpDown;
import frc.robot.commands.ElevatorCommands.PIDToHeight;
import frc.robot.commands.Targeting.AlgaeTarget;
import frc.robot.commands.Targeting.CoralLeftTarget;
import frc.robot.commands.Targeting.CoralRightTarget;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.commands.Targeting.TargetAngle;
import frc.robot.commands.Targeting.TargetAngleSide;
import frc.robot.commands.Targeting.TargetForwardDistance;
import frc.robot.commands.Targeting.TargetMegaTag2;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //Controllers
  XboxController driverController = new XboxController(Constants.Controller.USB_DRIVECONTROLLER);
  XboxController auxController = new XboxController(Constants.Controller.USB_AUXCONTROLLER);

   //AUTO SWITCHES
  private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
  private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
  private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
  private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);

   //Subsystems
  private final AlgaeHold  algaeHold = new AlgaeHold();
  private final AlgaePivot algaePivot = new AlgaePivot();
  private final Elevator elevator = new Elevator();
  private final CoralHold coralHold = new CoralHold();
  private final CoralPivot coralPivot = new CoralPivot(); 
  private final Swerve s_Swerve = new Swerve();

    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driverController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

   //COMMANDS

   //Drive
    private final OrientWithLL orientWithLL = new OrientWithLL(s_Swerve);

   //Targeting
    private final AlgaeTarget algaeTarget = new AlgaeTarget(s_Swerve);
    private final CoralLeftTarget coralLeftTarget = new CoralLeftTarget(s_Swerve);
    private final CoralRightTarget coralRightTarget = new CoralRightTarget(s_Swerve);

  //NOTE - STANDOFF FWD IS WITHOUT THE BUMPER - ADD BUMPER DEPTH AS NEEDEDD
    private final TargetAllParallel targetAllParallel = new TargetAllParallel(s_Swerve, 12, 0);
    private final TargetAngle targetAngle =  new TargetAngle(s_Swerve);
    private final TargetForwardDistance targetForwardDistance = new TargetForwardDistance(s_Swerve, 9);
    private final TargetSideDistance targetsideDistance = new TargetSideDistance(s_Swerve, 0);
    private final TargetSideDistance targetSideDistanceChanged  = new TargetSideDistance(s_Swerve,0);
    private final TargetMegaTag2 target3DMegaTag2 = new TargetMegaTag2(s_Swerve);
    private final TargetAngleSide targetAngleSide = new TargetAngleSide(s_Swerve, 0);

    //Auto
    private final DriveFwd driveFwdCenter55 = new DriveFwd(s_Swerve, false, 55);//88 + (3 for bumper) -36)
    private final TurnOnly turn = new TurnOnly(s_Swerve, false, -58);
    private final DriveFwdAndSideAndTurn driveFwdAndSideAndTurn = new DriveFwdAndSideAndTurn(s_Swerve, false, 9, 0, 0);

    private final FullRunRight fullRunRight = new FullRunRight(s_Swerve);
    private final Leg1Right leg1Right = new Leg1Right(s_Swerve);
    private final Leg2Right leg2Right = new Leg2Right(s_Swerve);
    private final Leg3Right leg3Right = new Leg3Right(s_Swerve);
    private final CtrScore1 fullRunCenter = new CtrScore1(s_Swerve);
    private final FullRunLeftCtr fullRunLeftCtr = new FullRunLeftCtr(s_Swerve);
   // private final DriveWithPath driveWithPathLeg1 = new DriveWithPath(s_Swerve, false);
    private final Legs1and2Right legs1and2Right = new Legs1and2Right(s_Swerve);
    
      
  //Elevator
  private final ManualUpDown elevatorUp = new ManualUpDown(elevator, Constants.Elevator.ELEV_UP_SPEED);
  private final ManualUpDown elevatorDown = new ManualUpDown(elevator, Constants.Elevator.ELEV_DOWN_SPEED);

  private final PIDToHeight pidElevToBottom = new PIDToHeight(elevator, Constants.Elevator.BOTTOM_HEIGHT);
  private final PIDToHeight pidElevatorL1 = new PIDToHeight(elevator, Constants.Elevator.L1_HEIGHT);
  private final PIDToHeight pidElevatorL2 = new PIDToHeight(elevator, Constants.Elevator.L2_HEIGHT);
  private final PIDToHeight pidElevatorL3 = new PIDToHeight(elevator, Constants.Elevator.L3_HEIGHT);
  private final PIDToHeight pidElevatorL4 = new PIDToHeight(elevator, Constants.Elevator.L4_HEIGHT);

  //AlgaeHold
  private final AlgaeGrab algaeGrab = new AlgaeGrab(algaeHold, Constants.AlgaeHold.HOLD_SPEED1, Constants.AlgaeHold.HOLD_SPEED2);
  private final AlgaeRelease algaeRelease = new AlgaeRelease(algaeHold, Constants.AlgaeHold.RELEASE_SPEED);
  private final AlgaeHighPickup algaeHighPickup = new AlgaeHighPickup(elevator, algaeHold, algaePivot);
  private final AlgaeLowPickup algaeLowPickup = new AlgaeLowPickup(elevator, algaeHold, algaePivot);

  //AlgaePivot
  private final ManualAlgaePivot algaePivotDown = new ManualAlgaePivot(algaePivot, Constants.AlgaePivot.MAN_EXT_SPEED);
  private final ManualAlgaePivot algaePivotUp = new ManualAlgaePivot(algaePivot, Constants.AlgaePivot.MAN_RET_SPEED);
  private final PIDAlgaePivot pidAlgaePickup = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ALGAE_PICKUP);
  private final PIDAlgaePivot pidAlgaeScoreNet = new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_SCORE_NET);

  //CoralHold
  private final CoralGrab coralGrab = new CoralGrab(coralHold, Constants.CoralHold.HOLD_SPEED);
  private final CoralGrabWithCounter coralGrabWithCounter = new CoralGrabWithCounter(coralHold, Constants.CoralHold.HOLD_SPEED);
  private final CoralRelease coralRelease = new CoralRelease(coralHold, Constants.CoralHold.RELEASE_SPEED);
  //private final CoralRelease coralReleaseL4 = new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED);

  //CoralPivot
  private final ManualCoralPivot coralPivotDown = new ManualCoralPivot(coralPivot, Constants.CoralPivot.MAN_EXT_SPEED);
  private final ManualCoralPivot coralPivotUp = new ManualCoralPivot(coralPivot, Constants.CoralPivot.MAN_RET_SPEED);
  private final PIDCoralPivot pidCoraltoL1 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL1);
  private final PIDCoralPivot pidCoraltoL2 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL2);
  private final PIDCoralPivot pidCoraltoL3 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL3);
  private final PIDCoralPivot pidCoraltoL4 = new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverController.getRawAxis(translationAxis), 
                () -> -driverController.getRawAxis(strafeAxis), 
                () -> -driverController.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Buttons
 zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    //Main Xbox Controller
    JoystickButton a = new JoystickButton(driverController, Constants.XboxController.A);
    JoystickButton b = new JoystickButton(driverController, Constants.XboxController.B);
      //Y on driver controller is assigned to "zeroGyro" above
   // JoystickButton y = new JoystickButton(driver, Constants.XboxController.Y);
   //leftBumper on driver controller is assigned to "robotCentric" above
   // JoystickButton lb = new JoystickButton(driver, Constants.XboxController.LB);
    // JoystickButton y = new JoystickButton(driverController, Constants.XboxController.Y);
    JoystickButton x = new JoystickButton(driverController, Constants.XboxController.X);
   // JoystickButton lb = new JoystickButton(driverController, Constants.XboxController.LB);
    JoystickButton rb = new JoystickButton(driverController, Constants.XboxController.RB);
    JoystickButton lm = new JoystickButton(driverController, Constants.XboxController.LM);
    JoystickButton rm = new JoystickButton(driverController, Constants.XboxController.RM);
    JoystickButton view = new JoystickButton(driverController, Constants.XboxController.VIEW);
    JoystickButton menu = new JoystickButton(driverController, Constants.XboxController.MENU);
    POVButton upPov = new POVButton(driverController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov = new POVButton(driverController,Constants.XboxController.POVXbox.DOWN_ANGLE); 
    POVButton leftPov = new POVButton(driverController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov = new POVButton(driverController,Constants.XboxController.POVXbox.RIGHT_ANGLE);
    
    //Secondary Xbox Controller
    JoystickButton x1 = new JoystickButton(auxController, Constants.XboxController.X);
    JoystickButton a1 = new JoystickButton(auxController, Constants.XboxController.A);
    JoystickButton b1 = new JoystickButton(auxController, Constants.XboxController.B);
    JoystickButton y1 = new JoystickButton(auxController, Constants.XboxController.Y);
    JoystickButton lb1 = new JoystickButton(auxController, Constants.XboxController.LB);
    JoystickButton rb1 = new JoystickButton(auxController, Constants.XboxController.RB);
    JoystickButton lm1 = new JoystickButton(auxController, Constants.XboxController.LM);
    JoystickButton rm1 = new JoystickButton(auxController, Constants.XboxController.RM);
    JoystickButton view1 = new JoystickButton(auxController, Constants.XboxController.VIEW);
    JoystickButton menu1 = new JoystickButton(auxController, Constants.XboxController.MENU);
    POVButton upPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.UP_ANGLE);
    POVButton downPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.DOWN_ANGLE);
    POVButton leftPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.LEFT_ANGLE);
    POVButton rightPov1 = new POVButton(auxController,Constants.XboxController.POVXbox.RIGHT_ANGLE);

    //Inputs

    //y button is already assigned to ZeroGyro
    //leftBumper lb button is already assigned to RobotCentric

    // a.whileTrue(coralLeftorAlgaeTarget);
    // b.whileTrue(coralRightTarget);
  
    // rb.whileTrue(targetAllParallel);
    // upPov.whileTrue(targetForwardDistance);
    // downPov.whileTrue(targetsideDistance);

      //a.whileTrue(legs1and2Right);
    //b.whileTrue(targetForwardDistance);
      //b.whileTrue(driveWithPathLeg1);
      //x.whileTrue(leg2Right);
      //upPov.whileTrue(leg1Right);
    //downPov.whileTrue(coralLeftTarget);
    //leftPov.whileTrue(driveFwdAndSideAndTurn);
    //leftPov.whileTrue(coralRightTarget);

    // a.whileTrue(elevatorDown);
    // x.whileTrue(elevatorUp);
   // b.onTrue(pidElevToBottom);
    // downPov.onTrue(pidElevatorL1);
    // leftPov.onTrue(pidElevatorL2);
    // upPov.onTrue(pidElevatorL3);
    //rightPov.onTrue(pidElevatorL4);

    // upPov.whileTrue(coralPivotUp);
    // downPov.whileTrue(coralPivotDown);
    // a.onTrue(pidCoraltoL1); //pivots coral to score on Level 1
    // b.onTrue(pidCoraltoL2);
    // leftPOV.onTrue(pidCoraltoL3);
    // rightPOV.onTrue(pidCoraltoL4);

    // rb.whileTrue(algaeGrab);
    // rm.whileTrue(algaeRelease);

    // a.whileTrue(coralGrab);
    // b.whileTrue(coralRelease);
    // leftPov.whileTrue(coralGrabWithCounter);

    upPov.whileTrue(algaePivotUp);
    downPov.whileTrue(algaePivotDown);
    a.onTrue(pidAlgaePickup); //pivot Algae to pickup from Reef position
    b.onTrue(pidAlgaeScoreNet); //pivot Algae to score in Net position


 // a.onTrue(driveFwdCenter55);
  //b.onTrue(turn);
  //upPov.onTrue(driveFwd113);
  //x.onTrue(fullRunRight);
  //rightPov.onTrue(fullRunLeftCtr);
  //a.onTrue(fullRunCenter);
  //x.onTrue(fullRunCenter);

  //downPov.whileTrue(algaeTarget);
  //leftPov.whileTrue(coralLeftTarget);
  //rightPov.whileTrue(coralRightTarget);
  }
  
  public Command getAutonomousCommand() {
   return null;//new FullRunRight(s_Swerve);
  }

}
