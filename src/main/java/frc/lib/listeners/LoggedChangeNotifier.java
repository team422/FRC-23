package frc.lib.listeners;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class LoggedChangeNotifier<T> extends ChangeNotifier<T> {
    private final String m_key;

    protected LoggedChangeNotifier(String key, Supplier<T> value) {
        super(value);
        m_key = key;
    }

    @Override
    protected void updateInternalState() {
        super.updateInternalState();
        Logger.getInstance().recordOutput(this.m_key, this.getValue().toString());
    }
}
