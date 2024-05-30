## Guide for using PlotJuggler
Download using: sudo apt install ros-humble-plotjuggler-ros
To start plotjuggler: ros2 run plotjuggler plotjuggler
Optional tags: -l <layout file> or -d <data file>

Current layout file: Experiment_0-5_Plots.xml

Then for bag files, create conversion file in the same directory using
cat << EOF > convert.yaml
output_bags:
 - uri: ros2_output
   storage_id: mcap
   all: true
   append: true
EOF
Then use: ros2 bag convert -i <rosbag file path> -o convert.yaml
This creates a mcap file in the ros2_output directory.

Once you have the mcap file, you can upload to plotjuggler using the data upload button in the top left corner.
