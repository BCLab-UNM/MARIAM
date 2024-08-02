# Resistor Tests with the Force Sensor
## Update: 8/2/24
These tests involved Ross pushing the object against the wall to see if the force measurement with the 1k Ohm resistor corresponds with the servos getting overloaded. Both of these tests exhibited force sensor saturation and an overloaded servo though it was not clear if the saturation directly corresponds in time with the overload because the arm did not give out and lose contact with the object as soon as it was overloaded.
![Force and Overload 1](images/ross_force_overload1.png)
![Force and Overload 2](images/ross_force_overload2.png)

Another test with Monica and Ross shows more clearly the difference in measurements between the two resistors, as well as the drift in Ross's measurements. The last part of this test where only Ross's measurements are present occurred because Monica was overloaded. Though it is still possible to get the full range of measurements for Ross, the 1k Ohm Resistor seems to better measure differences in the amount of force.
![Monica and Ross Resistor Comparison](images/monica_and_ross_force_comparison.png)

## Update: 8/1/24
According to the test below, having a smaller value resistor on Ross gave us a better range of values. In this test, Monica was overloaded, but Ross was not. Will need to test more to see if saturating the FSR corresponds with the servo overload.

### Test with 1k Ohm Resistor on Ross and 10k Ohm Resistor on Monica
![1k Ohm Resistor on Ross](images/1k_ohm_on_ross.png)

-----------------------------------------------------------------------------
The Interlink 406 Force Sensing Resistor does not appear to be sensitive enough for our experiments. We originally attached a 10k Ohm Resistor to the Teensy on both robots, but when looking at the force data in the ROS bags, it appeared to be at
its maximum value for most of the time that the robots were in contact with the object. I switched the resistor on Ross to a 100k Ohm one and then a 470k Ohm resistor, and the results were nearly the same. The force measurement is along the y-axis and time is along the bottom. 
### Test with 100k Ohm Resistor on Ross and 10k Ohm Resistor on Monica
![100k Ohm Resistor on Ross](images/100k_ohm_resistor.png "100k Ohm Resistor Test")


### Test with 470k Ohm Resistor on Ross and 10k Ohm Resistor on Monica
![470k Ohm Resistor on Ross](images/470k_ohm_resistor.png) 

### Test with 470k Ohm Resistor on Ross and varying weights
For this test, I used different weights placed on top of the force sensor (where the arm was pointing upwards). The most sloping force data came from the 50 g resistor, but due to the drift of the sensor, it was not stable. 
The force sensor was not able to detect force for all the weights less than 50 g (20g, 10 g, 5 g), and it maxed out for all the weights above 50 g (100 g, 200 g, 500 g, and 1k g) as soon as they were placed on it.
![Weights on Ross](images/weights_on_ross.png)
