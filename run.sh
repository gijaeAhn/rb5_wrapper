#!/bin/bash

# Start new tmux session
tmux new-session -d -s throwing

# Split into 8 panes
tmux split-window -h
tmux select-pane -L
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-pane -R

# Execute commands in left panes
tmux send-keys -t 0 'source /opt/ros/noetic/setup.bash && roscore' Enter
tmux send-keys -t 1 'sleep 1 && source ~/catkin_ws/devel/setup.bash && cd build/rb5_wrapper && ./cobot_controller ' Enter
tmux send-keys -t 2 'sleep 1 && source ~/catkin_ws/devel/setup.bash && cd build/rb5_wrapper &&  ./rb5_update' Enter
tmux send-keys -t 3 'sleep 1 && source ~/catkin_ws/devel/setup.bash && cd build/rb5_wrapper &&  ./rb5_wrapper' Enter

# Execute commands in right panes
tmux send-keys -t 4 'sleep 5 && source ~/catkin_ws/devel/setup.bash && cd build/rb5_wrapper &&  ./rb5_client' Enter
tmux send-keys -t 5 'sleep 2 && source ~/catkin_ws/devel/setup.bash && cd src/pythons && python3 commandPosition.py' Enter
tmux send-keys -t 6 'sleep 2 && source ~/catkin_ws/devel/setup.bash && cd src/pythons && python3 keyInput.py' Enter
tmux send-keys -t 7 'sleep 2 && source ~/catkin_ws/devel/setup.bash && cd src/pythons && python3 searchSever.py' Enter

# Attach to session
tmux attach-session -t throwing