package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {

    public TriggerButton(BooleanSupplier supplier) {
        super(() -> (supplier.getAsBoolean()));
    }
}
