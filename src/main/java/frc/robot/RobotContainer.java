package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.io.File;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
	List<PathPlannerTrajectory> closestPath;
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
	private final Ultrasonic intakeDistanceSensor = new Ultrasonic(0, 1);

	private final Field2d field = new Field2d();
	private final SendableChooser<Command> autonSelector = new SendableChooser<>();

	private SwerveAutoBuilder swerveAutoBuilder;

	public RobotContainer() {
		{
			HashMap<String, Command> EventMap = new HashMap<String, Command>();
			EventMap.put("ShootCubeTop", run(
					() -> {
						shooterSubsystem.setRotorSpeed(4.7, 9.1);
					}, shooterSubsystem)
					.withTimeout(1)
					.andThen(runOnce(() -> shooterSubsystem.stop(), shooterSubsystem)));
			EventMap.put("ShootCubeMid", run(
					() -> {
						shooterSubsystem.setRotorSpeed(3.4, 6);
					}, shooterSubsystem)
					.withTimeout(1)
					.andThen(runOnce(() -> shooterSubsystem.stop(), shooterSubsystem)));
			EventMap.put("Start Intake", run(() -> {
				if (intakeDistanceSensor.getRangeMM() > Constants.ShooterConstants.IntakeWidth) {
					shooterSubsystem.setFrontRotorSpeed(2.4);
				} else {
					shooterSubsystem.stop();
				}
			}, shooterSubsystem));
			EventMap.put("Stop Intake", run(() -> {
				shooterSubsystem.stop();
			}));

			swerveAutoBuilder = new SwerveAutoBuilder(m_robotDrive::getPose, m_robotDrive::resetOdometry,
					DriveConstants.kDriveKinematics, new PIDConstants(3, 0, 0),
					new PIDConstants(2, 0, 0), m_robotDrive::setModuleStates,
					EventMap, true, m_robotDrive);

			File ppFolder = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner/");
			System.out.println(ppFolder.getAbsolutePath());
			for (File file : ppFolder.listFiles()) {
				autonSelector.addOption(file.getName().replace(".path", ""),
						swerveAutoBuilder

								.fullAuto(PathPlanner.loadPathGroup(file.getName().replace(".path", ""), new PathConstraints(
										DriveConstants.kMaxSpeedMetersPerSecond * .75, DriveConstants.kDirectionSlewRate * .5))));
			}

			autonSelector.addOption("Shoot Top And Stay", run(
					() -> {
						shooterSubsystem.setRotorSpeed(4.7, 9.1);
					}, shooterSubsystem)
					.withTimeout(1)
					.andThen(runOnce(() -> shooterSubsystem.stop(), shooterSubsystem)));

			autonSelector.addOption("Shoot Top And Move To Charge",
					run(
							() -> {
								shooterSubsystem.setRotorSpeed(4.7, 9.1);
							}, shooterSubsystem)
							.withTimeout(.5)
							.andThen(run(() -> {
								shooterSubsystem.stop();
								m_robotDrive.drive(.4, 0, 0, true, true);
							}, m_robotDrive).until(() -> {
								return Math.abs(m_robotDrive.m_gyro.getPitch()) > 9;
							})).andThen(run(() -> {
								m_robotDrive.drive(.4, 0, 0, true, true);
							}, m_robotDrive).until(() -> {
								return Math.abs(m_robotDrive.m_gyro.getPitch()) < 3;
							}))
							.andThen(run(() -> {
								m_robotDrive.drive(.4, 0, 0, true, true);
							}, m_robotDrive).until(() -> {
								return Math.abs(m_robotDrive.m_gyro.getPitch()) > 9;
							}))
							.andThen(run(() -> {
								m_robotDrive.drive(.4, 0, 0, true, true);
							}, m_robotDrive).until(() -> {
								return Math.abs(m_robotDrive.m_gyro.getPitch()) < 3;
							}))
							.andThen(waitSeconds(1))
							.andThen(run(() -> {
								m_robotDrive.drive(-.5, 0, 0, true, true);
							}, m_robotDrive).until(() -> {
								return Math.abs(m_robotDrive.m_gyro.getPitch()) > 9;
							}))
							.andThen(waitSeconds(.2))
							.andThen(run(() -> {
								m_robotDrive.drive(-.3, 0, 0, true, true);
							}, m_robotDrive).until(() -> {
								return Math.abs(m_robotDrive.m_gyro.getPitch()) < 3;
							}))
							.andThen(run(() -> {
								m_robotDrive.setX();
							}, m_robotDrive))

			);

			autonSelector.addOption("Do Nothing", none());
			Shuffleboard.getTab("Autonomous").add("Auton Selector", autonSelector);
		}
		Ultrasonic.setAutomaticMode(true);
		configureButtonBindings();

		m_robotDrive.setDefaultCommand(
				run(
						() -> m_robotDrive.drive(
								-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
								-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
								true, true),
						m_robotDrive));

		Shuffleboard.getTab("Autonomous")
				.add("Field", field)
				.withWidget(BuiltInWidgets.kField);

	}

	private void configureButtonBindings() {
		m_driverController.x()
				.whileTrue(run(
						() -> m_robotDrive.setX(),
						m_robotDrive));

		m_driverController.leftBumper()
				.and(() -> {
					return intakeDistanceSensor.getRangeMM() > Constants.ShooterConstants.IntakeWidth;
				}).whileTrue(run(() -> shooterSubsystem.setFrontRotorSpeed(2.4), shooterSubsystem))
				.onFalse(waitSeconds(.02).finallyDo((boolean c) -> {
					shooterSubsystem.stop();
				}));

		m_driverController.rightBumper()
				.whileTrue(run(() -> shooterSubsystem.setFrontRotorSpeed(-2.4), shooterSubsystem)
						.finallyDo((boolean canceled) -> shooterSubsystem.stop()));
		m_driverController.leftTrigger().or(m_driverController.y())
				.whileTrue(run(() -> shooterSubsystem.setRotorSpeed(4.7, 9.1),
						shooterSubsystem)
						.finallyDo((boolean canceled) -> shooterSubsystem.stop()));

		m_driverController.rightTrigger().or(m_driverController.b())
				.whileTrue(run(() -> shooterSubsystem.setRotorSpeed(3.4, 6),
						shooterSubsystem)
						.finallyDo((boolean canceled) -> shooterSubsystem.stop()));

		m_driverController.start().onTrue(runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive));
	}

	public Command getAutonomousCommand() {
		return autonSelector.getSelected();
	}

	public void updateBoard() {
		Pose2d position = m_robotDrive.getPose();
		SmartDashboard.putNumber("GRYO PITCH", m_robotDrive.m_gyro.getPitch());
		field.setRobotPose(position);
		SmartDashboard.putNumber("dist", intakeDistanceSensor.getRangeMM());
	}
}
