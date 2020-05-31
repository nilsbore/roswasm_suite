SESSION=roswasm_webgui_example

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'roswasm_webgui'
tmux new-window -t $SESSION:2 -n 'joy_teleop'
tmux new-window -t $SESSION:3 -n 'joint_state_controller'
tmux new-window -t $SESSION:4 -n 'nodelet_tutorial_math'
tmux new-window -t $SESSION:5 -n 'video_stream_opencv'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "mon launch roswasm_webgui example_gui.launch --name=$(tmux display-message -p 'p#I_#W')" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch joy_teleop example.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch joint_state_controller joint_state_controller.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:4
tmux send-keys "mon launch nodelet_tutorial_math plus.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:5
tmux send-keys "mon launch video_stream_opencv camera.launch --name=$(tmux display-message -p 'p#I_#W')" C-m

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
