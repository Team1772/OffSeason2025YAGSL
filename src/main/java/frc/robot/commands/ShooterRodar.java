package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRodar extends Command {
    private final ShooterSubsystem shooter;


    public ShooterRodar(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
            }
       
    private void addRequirements(ShooterSubsystem shooter2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addRequirements'");
    }
   
    @Override
    public void initialize() {
        shooter.setMotores(0.8); // ou a velocidade desejada
    }
       
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotores();
    }
       
    @Override
    public boolean isFinished() {
        return false; // mantém rodando até soltar o botão
    }
}
