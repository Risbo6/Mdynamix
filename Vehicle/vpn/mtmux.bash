#!/bin/bash

TMUX_SESSION_NAME='mdynamix'

/usr/bin/tmux new -s $TMUX_SESSION_NAME=$ -d
/usr/bin/tmux send-keys -t $TMUX_SESSION_NAME=$ 'python3 /home/nvidia/Documents/vpn/vpn.py' Enter

echo "ok"
