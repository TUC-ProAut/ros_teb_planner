# teb_planner_pa_matlab package

## Older Matlab Version (before 2020b)

If you are running **Matlab 2020b** or newer have a look [here](README.md).
Otherwise stay here.

Usage instructions based on
  https://de.mathworks.com/matlabcentral/answers/283695 \
  "Updating existing custom message types with rosgenmsg"

### 1. Download package
Download this matlab package, which is part of the ProAut TEB-Planner
repository.

~~~~~
    e.g.
    $ mkdir -p ~/catkin_ws/src
    $ cd       ~/catkin_ws/src
    $ git clone https://github.com/TUC-ProAut/ros_teb_planner
    $ cd ros_teb_planner
    $ pwd

~~~~~

In the remainder of the instructions, it is assumed that the path
"~/catkin_ws/src/ros_teb_planner" is "REPO_PATH/".

### 2. Run rosgenmsg
Within matlab: run rosgenmsg on the folder containing the custom
message definitions.

~~~~~
    >> rosgenmsg('REPO_PATH/teb_planner_pa_matlab/msgs/')
~~~~~

In order to keep this instruction simple "MSGS_PATH" refers to
"REPO_PATH/teb_planner_pa_matlab/msgs/".

### 3. Edit the javaclasspath.txt
Follow the instructions to edit the javaclasspath.txt file.

In addition to the four JAR file paths, you also need to tell
Matlab to use these JAR files instead of the builtin ones. Add
the "**before**" token in front of the four JAR file paths:

~~~~~
    <before>
    MSGS_PATH/matlab_gen/jar/costmap_converter-0.0.11.jar
    MSGS_PATH/matlab_gen/jar/teb_local_planner-0.6.14.jar
    MSGS_PATH/matlab_gen/jar/teb_planner_pa_msgs-1.1.0.jar
    MSGS_PATH/matlab_gen/jar/visualization_msgs-1.12.7.jar
~~~~~

### 4. Restart Matlab
The caption says it all.

### 5. Regenerate files
Delete the previously generated Matlab files and run
rosgenmsg again. Now, it should pick up the new definitions:

~~~~~
    >> rmdir('MSGS_PATH/matlab_gen/', 's')
    >> rosgenmsg('MSGS_PATH')
~~~~~

### 6. done
You should now be able to use these new message definitions.
