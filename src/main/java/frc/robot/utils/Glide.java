package frc.robot.utils;

public class Glide {
    private long _gaintime;
    private double _last_output = 0.0;
    private double[] _gains;

    public Glide(double gain_up, double gain_down) 
    {
        _gains = new double[] {gain_up, gain_down};
        setGainTime();
    }

    public void setGainTime()
    {
        _gaintime = System.currentTimeMillis();
    }

    public double gain_adjust(double input)
    {
        long looptime = System.currentTimeMillis() - _gaintime;
        setGainTime();

        boolean toward_zero = Math.abs(_last_output) >  Math.abs(input);
        double distance = input - _last_output;
        double magnitude = Math.signum(distance);
        double maximum_distance = looptime * magnitude * _gains[toward_zero ? 1 : 0];

        if(Math.abs(maximum_distance) < Math.abs(distance))
            input = _last_output + maximum_distance;
        
        _last_output = input;

        return input;
    }
}
