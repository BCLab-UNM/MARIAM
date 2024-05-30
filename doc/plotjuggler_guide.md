## Guide for using PlotJuggler
Download using: 
```bash
sudo apt install ros-humble-plotjuggler-ros
```
To start plotjuggler: 
```bash
ros2 run plotjuggler plotjuggler
```
Optional tags: 
```bash
-l <layout file> or -d <data file>
```

Current layout file: Experiment_0-5_Plots.xml

Bag files must be converted to mcap files using the convert.yaml file. Create the file using this code:
```bash
cat << EOF > convert.yaml
output_bags:
 - uri: ros2_output
   storage_id: mcap
   all: true
   append: true
EOF
```
Then to convert bag files, run:
```bash 
ros2 bag convert -i <rosbag file path> -o convert.yaml
```
This creates a mcap file in the ros2_output directory.

Once you have the mcap file, you can upload to plotjuggler using the data upload button in the top left corner.
