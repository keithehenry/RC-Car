**RC Car**

*Controlling a two motor toy car with an RC airplane transmitter / receiver, Arduino (compatible) micro, and MX1919 motor driver.*

This code decodes the elevator and aileron PWM outputs of an RC receiver to control the speed and direction of a two motor RC car. The elevator axis is interpreted as the direction, forward or reverse, and speed. The aileron axis controls the turning radius. The computed values are them mapped into the four PWM signals required for the MX1919-based motor driver.
