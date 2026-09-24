#!/usr/bin/env bash
# start_explore.sh — 在 tmux 会话中启动三狗探索
# 在三只狗都初始化完成后运行

set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG=$SCRIPT_DIR/.log
mkdir -p "$LOG"
ENV_FILE="$LOG/ros_env.sh"
[ -f "$ENV_FILE" ] && source "$ENV_FILE"

echo "启动三狗探索..."

cd "$SCRIPT_DIR"
for i in 1 2 3; do
    BOX_ID=$((i == 1 ? 3 : (i == 2 ? 6 : 5)))
    python3 explore_simple.py \
        --box-json /home/ubuntu/go2_target_seek_delivery_0901/shadow_building_boxes.json \
        --box-id $BOX_ID --robot-name go2_$i \
        --action-server /go2_$i/navigate_to_pose --map-topic /go2_$i/map \
        --save-prefix "$LOG/explored_box_${BOX_ID}_go2_$i" \
        > $LOG/explore_$i.log 2>&1 &
    echo "  go2_$i 探索已启动 (Box $BOX_ID, PID $!)"
    sleep 2
done

echo ""
echo "探索已启动！监控面板会自动显示覆盖率。"
echo ""
echo "如果某只狗翻倒了："
echo "  1. 在 tmux 中 Ctrl+B 然后用方向键切换到该狗的窗格"
echo "  2. 按 R → 等2s → 0 → 等5s → 1 → 等3s → N"
echo "  3. 然后重新运行本脚本启动该狗的探索"
