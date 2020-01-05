package org.frc5687.deepspace.chassisbot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.RobotMap;

/**
 * Created by Ben Bernard on 2/2/2018.
 */
public class AutoChooser extends OutliersProxy {
    private RotarySwitch _modeSwitch;

    public AutoChooser(boolean isCompetitionBot) {
        _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH,  Constants.RotarySwitch.TOLERANCE, 0.07692, 0.15384, 0.23076, 0.30768, 0.3846, 0.46152, 0.53844, 0.61536, 0.69228, 0.7692, 0.84612, 0.92304);
    }



    public Mode getSelectedMode(){
        int raw = _modeSwitch.get();
        if (raw >= Mode.values().length) { raw = 0; }
        try {
            return Mode.values()[raw];
        } catch(Exception e){
                return Mode.StayPut;
        }
    }


    public void updateDashboard(){
        metric("Label/Mode", getSelectedMode().getLabel());
        metric("Raw/Mode", _modeSwitch.getRaw());
        metric("Numeric/Mode", _modeSwitch.get());
  }


    public enum Mode {
        StayPut(0, "Stay Put"),
        LeftDoubleRocket(1, "Left Double Rocket"),
        RightDoubleRocket(2, "Right Double Rocket");

        private String _label;
        private int _value;

        Mode(int value, String label) {
            _value = value;
            _label = label;
        }

        public int getValue() { return _value; }
        public String getLabel() { return _label; }

    }
}
