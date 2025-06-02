import subprocess
import multiprocessing
import os
import argparse
from datetime import datetime
from functools import partial

def run_sby(results_dir, task_id):
    """执行单个sby命令的函数"""
    sby_file = os.path.join(results_dir, f"SimTop_assert_verify_no{task_id}.sby")
    command = f"sby {sby_file} -f"
    
    print(f"🚀 Starting task no{task_id} (PID: {os.getpid()})")
    try:
        # 直接执行命令，不保存日志
        subprocess.run(command, shell=True, check=True)
        print(f"✅ Task no{task_id} completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Task no{task_id} failed with exit code {e.returncode}")
        return False
    except Exception as e:
        print(f"⚠️ Unexpected error in task no{task_id}: {str(e)}")
        return False

if __name__ == "__main__":
    # 设置命令行参数
    parser = argparse.ArgumentParser(description='并行执行SBY任务')
    parser.add_argument('--results-dir', default='results_E5', 
                        help='包含SBY文件的目录 (默认: results_E5)')
    parser.add_argument('--start-id', type=int, default=1, 
                        help='起始任务ID (默认: 1)')
    parser.add_argument('--end-id', type=int, default=43, 
                        help='结束任务ID (默认: 43)')
    parser.add_argument('--workers', type=int, default=multiprocessing.cpu_count(), 
                        help=f'并行工作进程数 (默认: CPU核心数)')
    
    args = parser.parse_args()
    
    # 创建任务ID列表
    tasks = list(range(args.start_id, args.end_id + 1))
    
    print(f"📂 SBY文件目录: {os.path.abspath(args.results_dir)}")
    print(f"🔢 任务范围: {args.start_id} - {args.end_id} (共 {len(tasks)} 个任务)")
    print(f"🚀 开始并行执行，使用 {args.workers} 个工作进程")
    
    # 使用部分函数固定参数
    run_task = partial(run_sby, args.results_dir)
    
    # 使用进程池并行执行
    with multiprocessing.Pool(processes=args.workers) as pool:
        results = pool.map(run_task, tasks)
    
    # 统计执行结果
    success_count = sum(results)
    failure_count = len(results) - success_count
    print("\n" + "="*50)
    print(f"📊 所有任务完成: {success_count} 成功, {failure_count} 失败")
    print("="*50)