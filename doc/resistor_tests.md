# Resistor Tests with the Force Sensor
The Interlink 406 Force Sensing Resistor does not appear to be sensitive enough for our experiments. We originally attached a 10k Ohm Resistor to the Teensy on both robots, but when looking at the force data in the ROS bags, it appeared to be at
its maximum value for most of the time that the robots were in contact with the object. I switched the resistor on Ross to a 100k Ohm one and then a 470k Ohm resistor, and the results were nearly the same. The force measurement is along the y-axis and time is along the bottom. 

### Test with 100k Ohm Resistor on Ross and 10k Ohm Resistor on Monica
![100k Ohm Resistor on Ross](images/100k_ohm_resistor.png "100k Ohm Resistor Test")


### Test with 470k Ohm Resistor on Ross and 10k Ohm Resistor on Monica
![470k Ohm Resistor on Ross](images/470k_ohm_resistor.png) 

### Test with 470k Ohm Resistor on Ross and varying weights
![Weights on Ross](images/weights_on_ross.png)
