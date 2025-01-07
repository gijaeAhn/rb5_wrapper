#!/bin/bash

# Start new tmux session
tmux new-session -d -s throwing

# Split into 8 panes
tmux split-window -h
tmux select-pane -L
tmux split-window -v
tmux select-pane -0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v
tmux select-pane -t 6
tmux split-window -v

tmux resize-pane -t 0 -x 50 -y 10
tmux resize-pane -t 1 -x 50 -y 10
tmux resize-pane -t 2 -x 50 -y 10
tmux resize-pane -t 3 -x 50 -y 10
tmux resize-pane -t 4 -x 50 -y 10
tmux resize-pane -t 5 -x 50 -y 10
tmux resize-pane -t 6 -x 50 -y 10
tmux resize-pane -t 7 -x 50 -y 10

# Execute commands in left panes
tmux send-keys -t 0 'source /opt/ros/noetic/setup.bash && roscore' Enter
tmux send-keys -t 1 'sleep 1 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper && ./cobot_controller ' Enter
tmux send-keys -t 2 'sleep 1 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper &&  ./rb5_update' Enter
tmux send-keys -t 3 'sleep 1 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper &&  ./rb5_wrapper' Enter

# Execute commands in right panes
tmux send-keys -t 4 'sleep 5 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper &&  ./rb5_client' Enter
tmux send-keys -t 5 'sleep 2 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper && python3 commandPosition.py' Enter
tmux send-keys -t 6 'sleep 2 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper && python3 keyInput.py' Enter
tmux send-keys -t 7 'sleep 2 && source ~/catkin_ws/devel/setup.bash && cd /home/ljw/Desktop/catkin_ws/devel/lib/rb5_ros_wrapper && python3 searchSever.py' Enter

# Attach to session
tmux attach-session -t throwing