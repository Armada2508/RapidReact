package frc.robot.Lib.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {

    private final AddressableLED mStrip;
    private final AddressableLEDBuffer mBuffer;
    private int mHue;
    private int mBandIndex;

    /**
     * Creates a new LEDStrip Object
     * @param port The PWM port that the LED strip is connected to
     * @param length The length of the LED strip
     */
    public LEDStrip(int port, int length) {
        mStrip = new AddressableLED(port);
        mBuffer = new AddressableLEDBuffer(length);
        mStrip.setLength(length);
        mStrip.setData(mBuffer);
        mStrip.start();
    }

    /**
     * Set the entire strip to one color
     * @param color The color to set the strip to
     */
    public void set(Color color) {
        set(0, mBuffer.getLength(), color);
    }

    /**
     * Set an led to a color
     * @param index The index of the LED to set
     * @param color The color to set the specified LED to
     */
    public void set(int index, Color color) {
        mBuffer.setLED(index, color);
        mStrip.setData(mBuffer);
    }
    
    /**
     * Set a range of the LED strip to a certain color
     * @param start The starting index
     * @param end The ending index
     * @param color The color to set the range to
     */
    public void set(int start, int end, Color color) {
        for (int i = start; i < end; i++) {
            mBuffer.setLED(i, color);
        }
        mStrip.setData(mBuffer);
    }
    
    /**
     * Set the LED strip to a certain color
     * @param h The hue of the color
     * @param s The saturation of the color
     * @param v The value of the color
     */
    public void setHSV(int h, int s, int v) {
        setHSV(0, mBuffer.getLength(), h, s, v);
    }
    
    /**
     * Set an LED to a certain color
     * @param index The index of the LED to set
     * @param h The hue of the color
     * @param s The saturation of the color
     * @param v The value of the color
     */
    public void setHSV(int index, int h, int s, int v) {
        mBuffer.setHSV(index, h, s, v);
        mStrip.setData(mBuffer);
    }
    
    /**
     * Set a range of the LED strip to a certain color
     * @param start The starting index
     * @param end The ending index
     * @param h The hue of the color
     * @param s The saturation of the color
     * @param v The value of the color
     */
    public void setHSV(int start, int end, int h, int s, int v) {
        for (int i = start; i < end; i++) {
            mBuffer.setHSV(i, h, s, v);
        }
        mStrip.setData(mBuffer);
    }


    /**
     * @return The length of the LED Strip
     */
    public int getLength() {
        return mBuffer.getLength();
    }

    /**
     * Get the color of an led
     * @param led The index of the LED
     * @return The color of the LED
     */
    public Color getColor(int index) {
        return mBuffer.getLED(index);
    }

    /**
     * Make a rainbow effect
     * @param increment The hue increment(speed)
     */
    public void rainbow(double increment) {
        rainbow(increment, 255, 255);
    }

    /**
     * Make a rainbow effect
     * @param increment The hue increment(speed)
     * @param s The saturation of the color
     * @param v The value(brightness) of the color
     */
    public void rainbow(double increment, int s, int v) {
        mHue += increment;
        mHue %= 180;
        setHSV(mHue, s, v);
    }

    /**
     * Make a rainbow band effect
     * @param increment The hue increment(speed)
     */
    public void rainbowBand(double increment) {
        rainbowBand(increment, 255, 255);
    }

    /**
     * Make a rainbow band effect
     * @param increment The hue increment(speed)
     * @param s The saturation of the color
     * @param v The value(brightness) of the color
     */
    public void rainbowBand(double increment, int s, int v) {
        mHue += increment;
        mHue %= 180;
        for (int i = 0; i < mBuffer.getLength(); i++) {
            int hue = (mHue + (i * 180 / mBuffer.getLength())) % 180;
            mBuffer.setHSV(i, hue, s, v);
        }
        mStrip.setData(mBuffer);
    }

    /**
     * Creates a band effect
     * @param increment How much the effect increments every time this method is called(speed)
     * @param color The color of the effect
     */
    public void band(double increment, Color color) {
        band(increment, color, 1);
    }

    /**
     * Creates a band effect
     * @param increment How much the effect increments every time this method is called(speed)
     * @param color The color of the effect
     * @param nodes How many nodes the effect has
     */
    public void band(double increment, Color color, int nodes) {
        mBandIndex += increment;
        for (int i = 0; i < mBuffer.getLength(); i++) {
            int halfWavelength = mBuffer.getLength() / (2 * nodes);
            int index = ( i + mBandIndex ) % mBuffer.getLength();
            int brightness = (halfWavelength - Math.abs(index % (2*halfWavelength) - halfWavelength) ) / halfWavelength;
            Color ledColor = new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
            mBuffer.setLED(index, ledColor);
        }
        mStrip.setData(mBuffer);
    }

    
}