#!/bin/bash

# 终止现有会话（如果存在）并启动新的 tmux 会话
if tmux has-session -t startdo 2>/dev/null; then
    tmux kill-session -t startdo
fi
tmux new-session -d -s startdo
tmux split-window -h -t startdo:0.0
tmux split-window -h -t startdo:0.1
tmux select-layout -t startdo even-horizontal

tmux select-pane -t startdo:0.0
tmux send-keys -t startdo:0.0 "source shegal/move_arm/install/setup.bash" C-m
tmux send-keys -t startdo:0.0 "source shegal/ty2_fk/install/setup.bash" C-m
tmux send-keys -t startdo:0.0 "ros2 run ty_kine tianyi_fkine_ros2" C-m

tmux select-pane -t startdo:0.1
tmux send-keys -t startdo:0.1 "source shegal/move_arm/install/setup.bash" C-m
tmux send-keys -t startdo:0.1 "KDL_CHAIN_SOURCE=urdf ros2 run move_box move_arm" C-m

tmux select-pane -t startdo:0.2
tmux send-keys -t startdo:0.2 "mkdir -p ~/xbags" C-m
tmux send-keys -t startdo:0.2 "cd ~/xbags" C-m
# 只录制 robot_controller 框架实际收发的 topic 和 service（不含大图像流）：
#   订阅: /arm/end_pose /finish_flag /error_flag 左右六维力 /robot_state /tf /tf_static
#   发布: /arm/mode /arm/cmd /head/cmd /leg/cmd /waist/cmd
#   服务: /arm/set_enable /arm/set_end_effector /arm/get_end_effector
tmux send-keys -t startdo:0.2 "ros2 bag record -o sport_bag_\$(date +%Y%m%d_%H%M%S) -d 600 --include-unpublished-topics --topics /arm/end_pose /finish_flag /error_flag /left_hand/dof6_force /right_hand/dof6_force /robot_state /tf /tf_static /arm/mode /arm/cmd /head/cmd /leg/cmd /waist/cmd --services /arm/set_enable /arm/set_end_effector /arm/get_end_effector" C-m

tmux attach -t startdo
