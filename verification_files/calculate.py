import os
import re
import argparse
from datetime import datetime

def find_earliest_fail(work_dir):
    """查找最早失败的任务日志并保存结果到文件"""
    # 存储所有失败任务的时间戳和文件夹信息
    fail_records = []
    
    # 正则表达式匹配时间戳和FAIL状态
    pattern = re.compile(
        r"SBY (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}).*DONE \(FAIL"
    )
    
    # 遍历no1到no43文件夹
    for i in range(1, 44):
        folder_name = f"SimTop_assert_verify_no{i}"
        log_path = os.path.join(work_dir, folder_name, "logfile.txt")
        
        # 检查日志文件是否存在
        if not os.path.exists(log_path):
            print(f"⚠️ 日志文件不存在: {log_path}")
            continue
        
        try:
            # 读取日志文件最后一行
            with open(log_path, 'r') as f:
                last_line = f.readlines()[-1].strip()
            
            # 检查是否包含FAIL
            if "FAIL" in last_line:
                # 提取时间戳
                match = pattern.search(last_line)
                if match:
                    timestamp_str = match.group(1)
                    # 转换为datetime对象
                    timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f")
                    fail_records.append((i, timestamp, timestamp_str))
                    print(f"✅ 发现失败任务 no{i}: {timestamp_str}")
                else:
                    print(f"⚠️ 无法解析时间戳: {last_line}")
        
        except Exception as e:
            print(f"❌ 处理 {log_path} 时出错: {str(e)}")
    
    # 如果没有失败任务
    if not fail_records:
        print("🎉 没有发现失败任务")
        return
    
    # 找出最早的时间戳
    earliest = min(fail_records, key=lambda x: x[1])
    task_id, _, timestamp_str = earliest
    
    # 创建结果字符串
    result = f"🔍 最早失败任务: no{task_id}\n⏱️ 时间戳: {timestamp_str}\n📂 工作目录: {os.path.abspath(work_dir)}"
    
    # 获取工作目录名（不含路径）
    dir_name = os.path.basename(os.path.abspath(work_dir))
    if not dir_name:  # 处理根目录情况
        dir_name = "root"
    
    # 生成结果文件名（工作目录名 + _result.txt）
    output_file = os.path.join(work_dir, f"{dir_name}_result.txt")
    
    try:
        with open(output_file, 'w') as f:
            f.write(result)
        print(f"\n💾 结果已保存至文件: {os.path.abspath(output_file)}")
    except Exception as e:
        print(f"❌ 保存结果失败: {str(e)}")
    
    # 控制台输出
    print("\n" + "="*60)
    print(result)
    print("="*60)

if __name__ == "__main__":
    # 设置命令行参数
    parser = argparse.ArgumentParser(description='查找最早失败的任务日志')
    parser.add_argument('--work-dir', default='.', 
                        help='工作目录路径 (默认: 当前目录)')
    
    args = parser.parse_args()
    
    # 确保目录存在
    if not os.path.exists(args.work_dir):
        os.makedirs(args.work_dir, exist_ok=True)
        print(f"📁 创建目录: {os.path.abspath(args.work_dir)}")
    
    find_earliest_fail(args.work_dir)