package frc.robot.commands.swervedrive.drivebase;
import edu.wpi.first.wpilibj2.command.Command;
 // Altere para o nome do seu subsistema
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SwerveModuleRotateToAngleCommand extends Command {
    private final SwerveSubsystem m_swerve;
    private final double m_angleDegrees;

    /**
     * Cria um novo SwerveModuleRotateToAngleCommand.
     *
     * @param swerve O subsistema Swerve.
     * @param angleDegrees O ângulo alvo em graus.
     */
    public SwerveModuleRotateToAngleCommand(SwerveSubsystem swerve, double angleDegrees) {
        m_swerve = swerve;
        m_angleDegrees = angleDegrees;
        addRequirements(m_swerve); // Garante o uso exclusivo do subsistema.
    }

    @Override
    public void initialize() {
        // Nada de especial para fazer aqui, a ação principal acontece em execute().
        System.out.println("Iniciando comando de rotação para " + m_angleDegrees + " graus.");
    }

    @Override
    public void execute() {
        // Chama o método setModuleAngle com o ângulo desejado.
        // A YAGSL cuida de toda a lógica para girar os módulos.
        m_swerve.getSwerveDrive().getModules();
        m_swerve.setModuleAngle(m_angleDegrees);
        // Opcional: pode adicionar log para depuração.
        // PathPlannerLogging.logCommand("SwerveModuleRotateToAngleCommand", "Rotating to " + m_angleDegrees + " degrees.");
    }

    @Override
    public void end(boolean interrupted) {
        // Nada de especial para fazer ao terminar. Os módulos irão manter a posição.
        System.out.println("Comando de rotação finalizado.");
    }

    @Override
    public boolean isFinished() {
        // O comando termina instantaneamente, pois o método setModuleAngle é imediato.
        // Se você precisar que ele espere até os módulos chegarem ao ângulo,
        // a lógica de verificação de 'isFinished' seria mais complexa.
        return true;
    }
}