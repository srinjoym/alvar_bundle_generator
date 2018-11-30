# alvar_bundle_generator
Automatically generates a bundle file based on tag locations for AR Track Alvar in ROS.

## Setup
This package depends on several python packages to work.

```
pip install numpy
pip install pyquaternion
pip install pybrain
```

## Running the package
The script expects two arguments, the first is the tag size(in cm) and the second is the id of the tag you want to be the master.

`rosrun alvar_bundle_generator <tag_size> <master_tag_id>`

After you get an appropriate number of images of the tags, press enter and the script will generate an xml file in the current directory.
