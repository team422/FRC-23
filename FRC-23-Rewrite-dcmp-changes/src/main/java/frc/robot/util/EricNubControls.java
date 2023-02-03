package frc.robot.util;

public class EricNubControls {
    public EricNubControls() {
    }

    public double addDeadzoneBasic(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        } else {
            return input;
        }
    }

    public double addDeadzoneScaled(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        } else {
            return (input - Math.signum(input) * deadzone) / (1 - deadzone);
        }
    }

    public double addEricCurve(double input) {
        if (input < 0.8) {
            return (1.0 / 3.0) * input + (1.0 / 9.0) * input * input + (65.0 / 72.0) * input * input * input;
        } else if (input> 0.0) {
            return (1.0 / 5.0) * Math.pow(5.0 * input - 5.0, 3) + 1.0;
        }else if (input> -0.8){
            return (1.0 / 3.0) * input + (1.0 / 9.0) * input * input + (65.0 / 72.0) * input * input * input;
        }else{
            return (1.0 / 5.0) * Math.pow(5.0 * input + 5.0, 3) - 1.0;
        }
    }
    // public void main(String[] args){
    //     EricNubControls u = new EricNubControls();

    // }
}
