#!/bin/bash

# PX4 Messages Cache 单元测试运行脚本
# 这个脚本编译并运行所有的单元测试，特别是多线程竞态场景测试

echo "=== PX4 Messages Cache 单元测试 ==="
echo "测试包括："
echo "1. 基本功能测试"
echo "2. 并发读写测试"
echo "3. 多线程写入竞态测试"
echo "4. 读取器饥饿测试"
echo "5. 高并发压力测试"
echo "6. 数据一致性验证测试"
echo ""

# 设置工作目录
cd /home/cfly/ros2_ws

# 构建项目
echo "正在构建项目..."
colcon build --packages-select px4_interface --cmake-args -DCMAKE_BUILD_TYPE=Debug

if [ $? -ne 0 ]; then
    echo "❌ 构建失败！"
    exit 1
fi

echo "✅ 构建成功！"

# 设置环境
source install/setup.bash

# 运行测试
echo ""
echo "正在运行单元测试..."
echo "⚠️  注意：部分测试涉及多线程竞态场景，可能需要较长时间完成"
echo ""

# 运行测试，并显示详细输出
colcon test --packages-select px4_interface --event-handlers console_direct+

# 检查测试结果
echo ""
echo "=== 测试结果总结 ==="
colcon test-result --all --verbose

echo ""
echo "测试完成！查看详细日志请检查 log/ 目录"
