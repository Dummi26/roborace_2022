#!/bin/fish
clear
cargo build --release && mv './target/armv5te-unknown-linux-musleabi/release/linienfolger' '/tmp/ev3dev/sshfs/home/robot/school/'