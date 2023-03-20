package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.TurretConstants.*;

public class PRotate extends ProfiledPIDCommand{
    private final Turret m_turret;
    public PRotate(double angle, Turret turret, CommandXboxController controller) {
        super(
            new ProfiledPIDController(COEFF.P, COEFF.I, COEFF.D, new TrapezoidProfile.Constraints(COEFF.MAX_ACC, COEFF.MAX_VEL)), 
            turret::position, 
            angle,  
            (output, setpoint) -> turret.turn(output), 
            turret);

        //getController().enableContinuousInput(-180, 180);
        getController().setTolerance(COEFF.ALLOWED_ERR, COEFF.MAX_RPM);  // max rpm is the allowed error of velocity. sorry for the sloppy writing

        m_turret = turret;
    }
    /**
     * Unsure if this is the right way to do this.
     * Unsure of how many limit switches we have.
     * The idea is it should stop going clockwise if it went 180 clockwise and it should stop going counterclockwise if it went -180
     */
    @Override
    public void initialize() {
        //if (m_turret.limit() && m_turret.position() > 0 && getController().getGoal().position > m_turret.position()) {end(true);}
        //else if (m_turret.limit() && m_turret.position() < 0 && getController().getGoal().position < m_turret.position()) {end(true);}
    }
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
