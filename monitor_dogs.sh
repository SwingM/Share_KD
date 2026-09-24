#!/usr/bin/env bash
# monitor_dogs.sh — tmux 监控面板：实时显示三狗状态
# 在 tmux pane 里运行

set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG=$SCRIPT_DIR/.log
SESSION="dogs"

# 确保 ROS 环境
ENV_FILE="$LOG/ros_env.sh"
[ -f "$ENV_FILE" ] && source "$ENV_FILE"

clear
echo "============================================================"
echo "  三狗状态监控面板"
echo "  Ctrl+C 退出此面板（不影响其他窗格）"
echo "============================================================"
echo ""

# ── 交互指南 ──
echo "┌──────────────────────────────────────────────────────┐"
echo "│  在左侧三个窗格中手动输入命令控制狗:                    │"
echo "│                                                      │"
echo "│  R = 重置    0 = 起身    1 = 策略    N = 导航          │"
echo "│                                                      │"
echo "│  完整序列: R → (等2s) → 0 → (等5s) → 1 → (等3s) → N  │"
echo "│                                                      │"
echo "│  翻倒了? 在对应窗格按 R 重置，再走一遍序列。            │"
echo "└──────────────────────────────────────────────────────┘"
echo ""

while true; do
    echo "$(date '+%H:%M:%S')"
    echo "────────────────────────────────────────────────"
    for i in 1 2 3; do
        # Joint states 检测（最可靠）
        JS=$(timeout 5 ros2 topic info /go2_$i/joint_states 2>/dev/null | grep -c "Publisher count: [1-9]")

        # IMU 姿态检测
        IMU=$(timeout 5 ros2 topic info /go2_$i/imu 2>/dev/null | grep -c "Publisher count: [1-9]")
        
        # Scan
        SCAN=$(timeout 5 ros2 topic info /go2_$i/scan 2>/dev/null | grep -c "Publisher count: [1-9]")

        # Coverage
        COV=$(grep -oP 'Coverage: \K[0-9.]+' $LOG/explore_$i.log 2>/dev/null | tail -1)
        EXPLORE_STATUS=$(tail -1 $LOG/explore_$i.log 2>/dev/null | grep -oP '\[.*?\]' | head -1)

        # rl_sim 状态
        RL_STATE="---"
        if tail -c 2000 $LOG/rlsim_$i.log 2>/dev/null | grep -qa "Navigation mode: ON"; then
            RL_STATE="NAV"
        elif tail -c 2000 $LOG/rlsim_$i.log 2>/dev/null | grep -qa "RLLocomotion"; then
            RL_STATE="RL"
        elif tail -c 2000 $LOG/rlsim_$i.log 2>/dev/null | grep -qa "GetUp"; then
            RL_STATE="GETUP"
        elif tail -c 2000 $LOG/rlsim_$i.log 2>/dev/null | grep -qa "resting"; then
            RL_STATE="REST"
        fi

        # 姿态判定
        if [ "$JS" -ge 1 ] && [ "$SCAN" -ge 1 ]; then
            POSE="UPRIGHT"
            COLOR="\033[32m"
        elif [ "$JS" -ge 1 ]; then
            POSE="STANDING(NO_SCAN)"
            COLOR="\033[33m"
        else
            POSE="DOWN"
            COLOR="\033[31m"
        fi

        printf "${COLOR}go2_%d:${RESET}  pose=%-16s  ctrl=%-3s  scan=%-3s  rl=%-4s  cov=%s%%\n" \
            "$i" "$POSE" "$([ $JS -ge 1 ] && echo Y || echo N)" \
            "$([ $SCAN -ge 1 ] && echo Y || echo N)" "$RL_STATE" "${COV:-N/A}"
    done
    echo "────────────────────────────────────────────────"

    # 检查探索是否全部完成
    DONE=0
    for i in 1 2 3; do
        if tail -5 $LOG/explore_$i.log 2>/dev/null | grep -q "Exploration finished\|Too many failures\|Coverage.*95"; then
            DONE=$((DONE+1))
        fi
    done
    if [ "$DONE" -ge 3 ]; then
        echo ""
        echo "🎉 所有探索已完成!"
        break
    fi

    sleep 10
done
