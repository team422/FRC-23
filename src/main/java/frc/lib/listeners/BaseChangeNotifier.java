package frc.lib.listeners;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.HashSet;
import java.util.Set;

public abstract class BaseChangeNotifier {
  private static Set<BaseChangeNotifier> changeNotifiers;

  protected static void registerChangeNotifier(BaseChangeNotifier changeNotifier) {
    requireNonNullParam(changeNotifier, "changeNotifier", "registerChangeNotifier");

    if (changeNotifiers == null) {
      changeNotifiers = new HashSet<>();
    }

    changeNotifiers.add(changeNotifier);
  }

  protected static void removeChangeListener(BaseChangeNotifier notifier) {
    if (changeNotifiers == null) {
      return;
    }

    changeNotifiers.remove(notifier);
  }

  public static final void updateAllChangeNotifiers() {
    if (changeNotifiers != null) {
      changeNotifiers.forEach(x -> x.update());
    }
  }

  protected abstract void updateInternalState();

  protected abstract boolean shouldNotify();

  protected abstract void notifyConsumers();

  private void update() {
    updateInternalState();
    if (shouldNotify()) {
      notifyConsumers();
    }
  }
}
