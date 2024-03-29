package util.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class provides a way to trigger commands with inputs
 * from the Dpad of a Joystick 
 */
public class JoystickDpad extends POVButton {
    private final Joystick joystick;
    private final int position;

    /**
     * Creates a Dpad button that can be used to trigger commands
     * 
     * @param joystick The joystick object that has the Dpad
     * @param position The position of the Dpad
     */
    public JoystickDpad(Joystick joystick, int position){
        super(joystick, position);
        this.joystick = joystick;
        if(position > 315 || position < -1) {
            this.position = 0;
            return;
        }
        this.position = position;
    }

    /**
     * @return Status of the POV of the joystick matching the POV provided
     */
    public boolean get(){
        return joystick.getPOV() == position;
    }
}
