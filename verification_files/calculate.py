import os
import re
import argparse
from datetime import datetime

def format_timedelta(td):
    """将时间差格式化为易读的字符串（天/小时/分钟/秒/毫秒）"""
    total_seconds = td.total_seconds()
    days = total_seconds // 86400
    hours = (total_seconds % 86400) // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return f"{int(days)}天{int(hours)}小时{int(minutes)}分钟{seconds:.3f}秒"

def find_earliest_fail(work_dir):
    """查找最早失败的任务日志并计算时间差"""
    # 存储所有失败任务的时间戳和文件夹信息
    fail_records = []
    # 存储所有任务的起始时间
    all_start_times = []
    
    # 正则表达式匹配时间戳
    start_pattern = re.compile(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})")
    fail_pattern = re.compile(
        r"SBY (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}).*DONE \(FAIL"
    )
    
    # 遍历no1到no43文件夹
    for i in range(1, 19):
        folder_name = f"SimTop_assert_verify_no{i}"
        log_path = os.path.join(work_dir, folder_name, "logfile.txt")
        
        # 检查日志文件是否存在
        if not os.path.exists(log_path):
            print(f"⚠️ 日志文件不存在: {log_path}")
            continue
        
        try:
            with open(log_path, 'r') as f:
                lines = f.readlines()
                
                # 解析第一行起始时间
                if lines:
                    first_line = lines[0].strip()
                    start_match = start_pattern.search(first_line)
                    if start_match:
                        start_str = start_match.group(1)
                        start_time = datetime.strptime(start_str, "%Y-%m-%d %H:%M:%S.%f")
                        all_start_times.append(start_time)
                        print(f"⏱️ 记录任务 no{i} 起始时间: {start_str}")
                    else:
                        print(f"⚠️ 无法解析起始时间: {first_line}")
                
                # 检查最后一行是否包含FAIL
                if lines:
                    last_line = lines[-1].strip()
                    if "FAIL" in last_line:
                        fail_match = fail_pattern.search(last_line)
                        if fail_match:
                            fail_str = fail_match.group(1)
                            fail_time = datetime.strptime(fail_str, "%Y-%m-%d %H:%M:%S.%f")
                            fail_records.append({
                                'task_id': i,
                                'start_time': start_time if 'start_time' in locals() else None,
                                'fail_time': fail_time,
                                'start_str': start_str if 'start_str' in locals() else "N/A",
                                'fail_str': fail_str
                            })
                            print(f"🔴 发现失败任务 no{i}: {fail_str}")
                        else:
                            print(f"⚠️ 无法解析失败时间戳: {last_line}")
        
        except Exception as e:
            print(f"❌ 处理 {log_path} 时出错: {str(e)}")
    
    # 如果没有失败任务
    if not fail_records:
        print("🎉 没有发现失败任务")
        return
    
    # 找出最早失败任务
    earliest_fail = min(fail_records, key=lambda x: x['fail_time'])
    
    # 计算全局最早起始时间
    if not all_start_times:
        print("⚠️ 未找到任何有效的起始时间")
        return
    
    global_earliest_start = min(all_start_times)
    global_start_str = global_earliest_start.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
    # 计算时间差
    time_diff = earliest_fail['fail_time'] - global_earliest_start
    time_diff_str = format_timedelta(time_diff)
    
    # 计算任务自身运行时间（如果起始时间可用）
    task_duration_str = "N/A"
    if earliest_fail['start_time']:
        task_duration = earliest_fail['fail_time'] - earliest_fail['start_time']
        task_duration_str = format_timedelta(task_duration)
    
    # 创建结果字符串
    result = (
        f"🔍 最早失败任务: no{earliest_fail['task_id']}\n"
        f"⏱️ 失败时间: {earliest_fail['fail_str']}\n"
        f"⏱️ 任务起始时间: {earliest_fail['start_str']}\n"
        f"⏱️ 任务运行时间: {task_duration_str}\n"
        f"🌍 全局最早起始时间: {global_start_str}\n"
        f"⏳ 起始到失败的时间差: {time_diff_str}\n"
        f"📂 工作目录: {os.path.abspath(work_dir)}"
    )
    
    # 获取工作目录名
    dir_name = os.path.basename(os.path.abspath(work_dir)) or "root"
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
    parser = argparse.ArgumentParser(description='查找最早失败的任务日志并计算时间差')
    parser.add_argument('--work-dir', default='.', 
                        help='工作目录路径 (默认: 当前目录)')
    args = parser.parse_args()
    
    if not os.path.exists(args.work_dir):
        os.makedirs(args.work_dir, exist_ok=True)
        print(f"📁 创建目录: {os.path.abspath(args.work_dir)}")
    
    find_earliest_fail(args.work_dir)