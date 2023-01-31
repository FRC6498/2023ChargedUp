package frc.robot;

public class cowCatcher {

  DoubleSolenoid pushcatcher = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.Shifter_Forward_Channel, DriveConstants.Shifter_Reverse_Channel);

  public Command PushCone() {
    return runOnce(this::pushcatcher).andThen(Commands.waitSeconds(0.5)).andThen(runOnce(this::pushcatcher));
  }

  public void pushcatcher() {
    boolean  extracted = false; 
    
    if (pushcatcher.get() == Value.kOff) {
      extracted= false;
    }
    else (pushcatcher.get() == Value.kForward) {
      extracted= true;
    }

    if (extracted) {
      pushcatcher.set(Value.kOff);
      extracted= false;
    } else {
      pushcatcher.set(Value.kForward);
      extracted = true;
    }
  }
}
