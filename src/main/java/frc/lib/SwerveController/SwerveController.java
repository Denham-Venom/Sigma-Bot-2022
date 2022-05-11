package frc.lib.SwerveController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SwerveController extends CommandBase {
    private final Timer m_timer = new Timer();
    private Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final SwerveTrajectory s_config;
    private List<List<Double>> timedHeadings = new ArrayList<List<Double>>();
    private int headingCounter = 0;

    public SwerveController(SwerveTrajectory s_config) {
        this.s_config = s_config;
        m_pose = s_config.getControllerConfig().m_pose;
        m_kinematics = s_config.getControllerConfig().m_kinematics;
        m_outputModuleStates = s_config.getControllerConfig().m_outputModuleStates;

        Constraints kThetaControllerConstraints =
            new Constraints(s_config.getControllerConfig().maxVelocity, s_config.getControllerConfig().maxAcceleration);

        ProfiledPIDController thetaController = 
            new ProfiledPIDController(s_config.getControllerConfig().kPThetaController, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_controller = new HolonomicDriveController(
            new PIDController(s_config.getControllerConfig().kPXController, 0, 0), 
            new PIDController(s_config.getControllerConfig().kPYController, 0, 0), 
            thetaController
        );

        addRequirements(s_config.getControllerConfig().requirements);
    }

    @Override
    public void initialize() {
        /* Generate trajectory */
        List<Pose2d> poseList = new ArrayList<>();
        for(SwerveWayPoint wp : s_config.getWayPoints()){
            poseList.add(wp.getPose());
        };

        m_trajectory = TrajectoryGenerator.generateTrajectory(
            poseList,
            s_config.getTrajectoryConfig()
        );
        
        /* Generate List of timed headings */
        int i = 0;
        for (SwerveWayPoint wp : s_config.getWayPoints()) {
            for ( ; i < m_trajectory.getStates().size(); i++) {
                if (wp.getPose().equals(m_trajectory.getStates().get(i).poseMeters)){
                    List<Double> list = new ArrayList<>();
                    list.add(m_trajectory.getStates().get(i).timeSeconds);
                    list.add(wp.getRobotHeading().getDegrees());
                    timedHeadings.add(list);
                    break;
                }
            }
        }

        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        /* Fetch desired heading using current time */
        if ((curTime > timedHeadings.get(headingCounter).get(0)) && (headingCounter < timedHeadings.size() - 1)) {
            headingCounter += 1;
        }

        Rotation2d targetHeading = Rotation2d.fromDegrees(timedHeadings.get(headingCounter).get(1));

        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, targetHeading);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_outputModuleStates.accept(m_kinematics.toSwerveModuleStates(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
