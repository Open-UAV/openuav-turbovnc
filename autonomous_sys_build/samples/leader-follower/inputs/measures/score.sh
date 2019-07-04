#!/usr/bin/env bash
cat /simulation/leader-follower/outputs/measure.csv | awk -F',' '{sum+=$2; ++n} END { print sum/n }' > /simulation/leader-follower/outputs/score.out
cat /simulation/leader-follower/outputs/score.out