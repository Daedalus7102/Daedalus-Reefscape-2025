package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification.NotificationLevel;

public class ElevatorIO extends SubsystemBase{
    Elastic.Notification notification = new Elastic.Notification();

    public ElevatorIO() {
        Elastic.sendNotification(notification
            .withLevel(NotificationLevel.ERROR)
            .withTitle("Some Information")
            .withDescription("Your robot is doing fantastic!")
            .withDisplaySeconds(5.0)
        );
        
        Elastic.sendNotification(notification);
    
    }
}
