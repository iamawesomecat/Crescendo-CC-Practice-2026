package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;

public class Superstructure {
    private final Indexer indexer;
    private final Flywheels flywheels;
    private final Pivot pivot;

    public Superstructure(Indexer indexer, Flywheels flywheels, Pivot pivot) {
        this.indexer = indexer;
        this.flywheels = flywheels;
        this.pivot = pivot;
    }

    public Command stationIntake(){
        if(indexer != null){
            return Commands.parallel(indexer.setPowerCommand(0.5), pivot.setTargetPosition(45));
        } else {
            return Commands.none().withName("indexer disabled");
        }
    }
}
