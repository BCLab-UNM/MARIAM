# mariam_description

This ROS2 Humble package holds all of the files describing the MARIAM agents.

## Gazebo Model Structure

By adding the folder `~/MARIAM/src/mariam_description/models/` to your environment variable `$GAZEBO_MODEL_PATH`, Gazebo will allow you to drop in models manually from the *insert* tab in the Gazebo application window. However the models inside of this folder must be of a certain structure:

1. Create a folder inside of `~/MARIAM/src/mariam_description/models/` called `<name of model>_description
2. Copy the URDF or SDF into this folder
3. Create a a `model.config` inside this subfolder

There are two models with this feature in the `mariam_description` package: `mariam_description` and `px100_modified_description`.

## Other Models

All other models are simply stored within `~/MARIAM/src/mariam_description/models/` without any necessary structure.