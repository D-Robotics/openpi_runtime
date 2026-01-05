#!/bin/bash

# 交互式录制脚本 - 修复版
# 功能：按 s 开始录制，按 d 停止录制，按 q 退出程序

# 定义要录制的话题
TOPICS=(
    "/aliciaD/action"
    "/piper/qpos" 
    "/camera/camera/color/image_raw"
    "/camera_left/camera_left/color/image_raw"
)

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 状态变量
is_recording=false
bag_pid=""
bag_name=""
start_time=0
stop_timeout=3  # 停止录制的超时时间（秒）

# 获取话题列表（修复管道错误）
get_topic_list() {
    # 使用临时文件避免管道错误
    local tmp_file="/tmp/ros2_topics_$$.tmp"
    ros2 topic list 2>/dev/null > "$tmp_file"
    if [ $? -ne 0 ]; then
        echo -e "${RED}错误: 无法获取话题列表，请确保ROS2环境已设置${NC}"
        rm -f "$tmp_file" 2>/dev/null
        return 1
    fi
    cat "$tmp_file"
    rm -f "$tmp_file" 2>/dev/null
    return 0
}

# 清理函数
cleanup() {
    echo -e "\n${CYAN}正在清理...${NC}"
    
    # 如果正在录制，先停止
    if [ "$is_recording" = true ] && [ ! -z "$bag_pid" ]; then
        if kill -0 $bag_pid 2>/dev/null; then
            echo -e "${YELLOW}检测到录制进程仍在运行，正在停止...${NC}"
            kill -INT $bag_pid 2>/dev/null
            
            # 等待最多3秒
            local count=0
            while kill -0 $bag_pid 2>/dev/null && [ $count -lt $stop_timeout ]; do
                sleep 0.5
                ((count++))
            done
            
            # 如果还没停止，强制终止
            if kill -0 $bag_pid 2>/dev/null; then
                echo -e "${RED}强制终止录制进程${NC}"
                kill -9 $bag_pid 2>/dev/null
            fi
        fi
    fi
    
    # 清理临时文件
    rm -f /tmp/ros2_topics_*.tmp 2>/dev/null
    echo -e "${GREEN}清理完成${NC}"
}

# 设置退出信号捕获
trap cleanup SIGINT SIGTERM EXIT

# 显示帮助信息
show_help() {
    clear
    echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║     ROS2 Bag 录制工具 (修复版)         ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
    echo -e "${CYAN}监控的话题:${NC}"
    for topic in "${TOPICS[@]}"; do
        echo -e "  ${GREEN}›${NC} $topic"
    done
    echo -e "${BLUE}══════════════════════════════════════════${NC}"
    echo -e "${YELLOW}按键说明:${NC}"
    echo -e "  ${GREEN}[s]${NC} 开始/重新开始录制"
    echo -e "  ${RED}[d]${NC} 停止录制"
    echo -e "  ${BLUE}[q]${NC} 退出程序"
    echo -e "  ${YELLOW}[h]${NC} 显示帮助"
    echo -e "  ${CYAN}[c]${NC} 检查话题状态"
    echo -e "${BLUE}══════════════════════════════════════════${NC}"
    
    if [ "$is_recording" = true ]; then
        local current_time=$(date +%s)
        local duration=$((current_time - start_time))
        local hours=$((duration / 3600))
        local minutes=$(( (duration % 3600) / 60 ))
        local seconds=$((duration % 60))
        
        echo -e "${RED}● 录制中${NC} | 时长: ${YELLOW}${hours}:${minutes}:${seconds}${NC}"
        echo -e "文件: ${CYAN}$bag_name${NC}"
    else
        echo -e "${GREEN}○ 空闲${NC} | 等待输入..."
    fi
    echo -e "${BLUE}══════════════════════════════════════════${NC}"
    echo -n "请选择操作 (s/d/q/h/c): "
}

# 检查话题是否存在
check_topics() {
    echo -e "${CYAN}检查话题状态...${NC}"
    
    # 获取话题列表
    local topic_list
    topic_list=$(get_topic_list)
    if [ $? -ne 0 ]; then
        return
    fi
    
    local found_count=0
    local missing_count=0
    
    for topic in "${TOPICS[@]}"; do
        if echo "$topic_list" | grep -q "^$topic$"; then
            echo -e "  ${GREEN}✓${NC} $topic"
            ((found_count++))
        else
            echo -e "  ${RED}✗${NC} $topic ${YELLOW}(未找到)${NC}"
            ((missing_count++))
        fi
    done
    
    echo -e "${CYAN}-----------------------------------------${NC}"
    echo -e "找到: ${GREEN}$found_count${NC} 个 | 缺失: ${RED}$missing_count${NC} 个"
    
    if [ $missing_count -eq ${#TOPICS[@]} ]; then
        echo -e "${RED}警告: 未找到任何话题！${NC}"
        echo -e "${YELLOW}请确保:${NC}"
        echo -e "  1. ROS2环境已设置 (source /opt/ros/jazzy/setup.bash)"
        echo -e "  2. 相关节点已启动"
        echo -e "  3. 话题名称拼写正确"
    fi
}

# 快速检查关键话题
quick_check_topics() {
    local topic_list
    topic_list=$(get_topic_list 2>/dev/null)
    
    local available_topics=()
    for topic in "${TOPICS[@]}"; do
        if echo "$topic_list" | grep -q "^$topic$"; then
            available_topics+=("$topic")
        fi
    done
    
    if [ ${#available_topics[@]} -eq 0 ]; then
        return 1
    fi
    
    echo "${available_topics[@]}"
    return 0
}

# 开始录制函数
start_recording() {
    if [ "$is_recording" = true ]; then
        echo -e "${RED}已在录制中，请先停止当前录制${NC}"
        return
    fi
    
    # 快速检查话题
    echo -e "${CYAN}检查话题可用性...${NC}"
    local available_topics
    available_topics=$(quick_check_topics)
    
    if [ $? -ne 0 ] || [ -z "$available_topics" ]; then
        echo -e "${RED}错误: 未找到任何可用话题！${NC}"
        echo -e "${YELLOW}请先检查话题状态 [c] 或启动相关节点${NC}"
        return
    fi
    
    # 生成带时间戳的文件名
    current_time=$(date +"%Y%m%d_%H%M%S")
    bag_name="bag_${current_time}"
    
    echo -e "${GREEN}开始录制...${NC}"
    echo -e "${CYAN}保存到:${NC} $bag_name"
    echo -e "${CYAN}录制话题:${NC}"
    for topic in $available_topics; do
        echo -e "  ${GREEN}›${NC} $topic"
    done
    
    # 在后台开始录制
    echo -e "${YELLOW}启动录制进程...${NC}"
    
    # 使用nohup避免信号干扰
    nohup ros2 bag record -o "$bag_name" $available_topics > "/tmp/ros2_bag_$$.log" 2>&1 &
    bag_pid=$!
    
    # 等待进程启动
    sleep 0.5
    
    if kill -0 $bag_pid 2>/dev/null; then
        is_recording=true
        start_time=$(date +%s)
        echo -e "${GREEN}✓ 录制已开始 (PID: $bag_pid)${NC}"
        echo -e "${YELLOW}提示: 按 [d] 停止录制${NC}"
    else
        echo -e "${RED}✗ 录制启动失败${NC}"
        bag_pid=""
    fi
}

# 停止录制函数
stop_recording() {
    if [ "$is_recording" = false ] || [ -z "$bag_pid" ]; then
        echo -e "${YELLOW}当前没有在录制${NC}"
        return
    fi
    
    echo -e "${YELLOW}正在停止录制...${NC}"
    
    # 检查进程是否存在
    if ! kill -0 $bag_pid 2>/dev/null; then
        echo -e "${RED}录制进程已不存在${NC}"
        is_recording=false
        bag_pid=""
        return
    fi
    
    # 发送SIGINT信号（相当于Ctrl+C）
    kill -INT $bag_pid 2>/dev/null
    
    # 等待进程结束
    local count=0
    while kill -0 $bag_pid 2>/dev/null && [ $count -lt $stop_timeout ]; do
        echo -n "."
        sleep 0.5
        ((count++))
    done
    
    echo ""  # 换行
    
    if kill -0 $bag_pid 2>/dev/null; then
        echo -e "${RED}警告: 进程未正常结束，强制终止${NC}"
        kill -9 $bag_pid 2>/dev/null
    else
        echo -e "${GREEN}✓ 录制已停止${NC}"
    fi
    
    # 等待一下确保文件写入完成
    sleep 0.5
    
    # 显示录制信息
    if [ -d "$bag_name" ]; then
        echo -e "${CYAN}录制信息:${NC}"
        echo -e "  ${GREEN}位置:${NC} $(pwd)/$bag_name"
        echo -e "  ${GREEN}格式:${NC} MCAP (.mcap文件)"
        echo -e "  ${GREEN}大小:${NC} $(du -sh "$bag_name" 2>/dev/null | cut -f1)"
        
        # 尝试获取bag信息（不显示错误）
        ros2 bag info "$bag_name" 2>/dev/null | head -20
    fi
    
    # 重置状态
    is_recording=false
    bag_pid=""
    
    # 显示日志文件位置
    if [ -f "/tmp/ros2_bag_$$.log" ]; then
        echo -e "${YELLOW}详细日志:${NC} /tmp/ros2_bag_$$.log"
    fi
}

# 关于MCAP格式的说明
show_mcap_info() {
    echo -e "${PURPLE}关于MCAP格式:${NC}"
    echo -e "${CYAN}✓${NC} MCAP是ROS2 Jazzy的默认bag格式"
    echo -e "${CYAN}✓${NC} 完全兼容ros2 bag命令"
    echo -e "${CYAN}✓${NC} 使用: ${GREEN}ros2 bag play $bag_name${NC}"
    echo -e "${CYAN}✓${NC} 查看信息: ${GREEN}ros2 bag info $bag_name${NC}"
    echo -e "${CYAN}✓${NC} 支持高效压缩和流式传输"
}

# 主程序
main() {
    # 检查ROS2环境
    if ! type ros2 >/dev/null 2>&1; then
        echo -e "${RED}错误: 未找到ros2命令${NC}"
        echo -e "${YELLOW}请先设置ROS2环境:${NC}"
        echo -e "  source /opt/ros/jazzy/setup.bash"
        exit 1
    fi
    
    echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║       ROS2 交互式录制工具             ║${NC}"
    echo -e "${BLUE}║             (ROS2 Jazzy)               ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
    
    # 显示MCAP格式说明
    show_mcap_info
    
    # 初始检查
    check_topics
    
    # 主循环
    while true; do
        show_help
        
        # 读取单个字符
        read -rsn1 input
        
        case $input in
            s|S)
                echo -e "\n${GREEN}[操作] 开始录制${NC}"
                start_recording
                ;;
            d|D)
                echo -e "\n${RED}[操作] 停止录制${NC}"
                stop_recording
                ;;
            q|Q)
                echo -e "\n${BLUE}[操作] 退出程序${NC}"
                if [ "$is_recording" = true ]; then
                    echo -e "${YELLOW}正在停止录制...${NC}"
                    stop_recording
                fi
                break
                ;;
            h|H)
                echo -e "\n${CYAN}[操作] 显示帮助${NC}"
                # show_help会在循环开始时显示
                ;;
            c|C)
                echo -e "\n${YELLOW}[操作] 检查话题状态${NC}"
                check_topics
                echo -e "\n${YELLOW}按任意键继续...${NC}"
                read -n1
                ;;
            *)
                if [ ! -z "$input" ]; then
                    echo -e "\n${RED}未知命令: $input${NC}"
                    echo -e "可用命令: s(开始), d(停止), q(退出), h(帮助), c(检查)"
                    sleep 1
                fi
                ;;
        esac
    done
    
    echo -e "${GREEN}感谢使用！${NC}"
}

# 运行主程序
main