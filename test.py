import os
import shutil
import subprocess
import argparse

def run_command(command, shell=False):
    try:
        process = subprocess.Popen(command, shell=shell, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        return_code = process.wait()
        return return_code
    except subprocess.CalledProcessError as e:
        print(f"Error occurred: {e.stderr}")
        return None
    except subprocess.TimeoutExpired as e:
        print(f"Timeout occurred: {e.stderr}")
        return None
    except Exception as e:
        print(f"Exception occurred: {e}")
        return None

def run_emu(args):
    run_path = os.path.join(current_dir, "tmp", "fuzz_run", f"{args.fuzz_id}")
    if os.path.exists(run_path):
        shutil.rmtree(run_path)
    os.makedirs(run_path, exist_ok=True)

    if args.dump_csr:
        csr_trans_path = os.path.join(run_path, "csr_transition")
        csr_wave_path = os.path.join(run_path, "csr_wave")
        os.mkdir(csr_trans_path)
        os.mkdir(csr_wave_path)
    
    commands = "./build/fuzzer"
    commands += f" -c firrtl.{args.cover_type}"
    commands += f" -- {args.image}"
    commands += f" -I {args.max_instr}"
    commands += f" -C {args.max_circle}"
    commands += f" --fuzz-id {args.fuzz_id}"

    if not args.no_dump_wave:
        commands += " --dump-wave-full"
        commands += f" --wave-path {args.wave_path}"
    
    if args.dump_trace:
        commands += " --dump-trace"
    if args.dump_csr:
        commands += " --dump-csr-change"
    
    commands += f" > {args.output_file} 2>&1"
    
    print(f"Commands: {commands}")
    ret = run_command(commands, shell=True)
    print(f"Return code: {ret}")

def run_fuzz(args):
    pass

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.realpath(__file__))
    os.chdir(current_dir)

    parser = argparse.ArgumentParser()
    
    # emu
    parser.add_argument("--emu", "-e", action='store_true', help="Run emulator")
    parser.add_argument("--image", "-i", type=str, default="tmp/bin/test.bin", help="Image file")
    parser.add_argument("--dump-trace", action='store_true', help="Dump trace")
    parser.add_argument("--dump-csr", action='store_true', help="Dump CSR")
    
    # fuzz
    parser.add_argument("--fuzz", "-f", action='store_true', help="Run fuzz")

    # run options
    parser.add_argument("--cover-type", "-c", type=str, default="toggle", help="Cover type")

    parser.add_argument("--max-circle", type=int, default=10000, help="Max circle")
    parser.add_argument("--max-instr", type=int, default=1000, help="Max instr")   

    parser.add_argument("--no-dump-wave", action='store_true', help="No dump wave")
    parser.add_argument("--wave-path", type=str, default="tmp/run_wave.vcd", help="Wave file")

    parser.add_argument("--fuzz-id", type=int, default=0, help="Fuzz id")

    parser.add_argument("--output-file", "-o", type=str, default="tmp/test.log", help="Output file")

    args = parser.parse_args()

    if args.emu:
        run_emu(args)
    elif args.fuzz:
        run_fuzz(args)
    