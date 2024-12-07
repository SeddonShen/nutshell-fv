import os
import shutil
import subprocess
import argparse

current_dir = os.getenv("NOOP_HOME")

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

    if args.use_asm_test:
        tmp_test_bin = os.path.join(current_dir, "tmp", "bin", "test.bin")
        asm_test_bin = os.path.join(current_dir, "ccover", "asms", "test.bin")
        if os.path.exists(tmp_test_bin):
            os.remove(tmp_test_bin)
        if not os.path.exists(asm_test_bin):
            print(f"Error: {asm_test_bin} not exists")
            return
        shutil.copy(asm_test_bin, tmp_test_bin)

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

    if args.run_snapshot:
        commands += " --run-snapshot"

    if args.no_diff:
        commands += " --no-diff"

    if not args.no_dump_wave:
        commands += " --dump-wave-full"
        commands += f" --wave-path {args.wave_path}"

    if args.dump_trace:
        commands += " --dump-commit-trace"
        commands += " --dump-ref-trace"
    if args.dump_csr:
        commands += " --dump-csr-change"

    if args.dump_footprints:
        # footprints_path = os.path.join(run_path, "footprints")
        # os.makedirs(footprints_path)
        commands += f" --dump-footprints {args.footprints_path}"
    

    if args.dump_trace:
        commands += f" > {args.output_file}"
        commands += f" 2> {args.err_file}"
    else:
        commands += f" > {args.output_file} 2>&1"
    
    print(f"Commands: {commands}")
    ret = run_command(commands, shell=True)
    print(f"Return code: {ret}")

def run_fuzz(args):
    pass

if __name__ == "__main__":
    os.chdir(current_dir)

    parser = argparse.ArgumentParser()

    # default
    default_max_circle = 500
    default_max_instr = 100
    default_fuzz_id = 0
    
    default_image = os.path.join(current_dir, "tmp", "bin", "test.bin")
    default_footprints_path = os.path.join(current_dir, "tmp", "fuzz_run", "0", "footprints")
    default_wave_path = os.path.join(current_dir, "tmp", "run_wave.vcd")
    default_output_file = os.path.join(current_dir, "tmp", "test.log")
    default_err_file = os.path.join(current_dir, "tmp", "test_err.log")
    
    # emu
    parser.add_argument("--emu", "-e", action='store_true', help="Run emulator")
    parser.add_argument("--image", "-i", type=str, default=default_image, help="Image file")
    parser.add_argument("--dump-trace", "-dt", action='store_true', help="Dump trace")
    parser.add_argument("--dump-csr", "-dc", action='store_true', help="Dump CSR")
    parser.add_argument("--dump-footprints", "-df", action='store_true', help="Dump footprints")
    parser.add_argument("--footprints-path", "-fp", type=str, default=default_footprints_path, help="Footprints path")

    parser.add_argument("--use-asm-test", "-ua", action='store_true', help="Use asm test bin")
    
    # fuzz
    parser.add_argument("--fuzz", "-f", action='store_true', help="Run fuzz")

    # run options
    parser.add_argument("--cover-type", "-c", type=str, default="toggle", help="Cover type")

    parser.add_argument("--no-diff", "-n", action='store_true', help="No diff")

    parser.add_argument("--max-circle", type=int, default=default_max_circle, help="Max circle")
    parser.add_argument("--max-instr", type=int, default=default_max_instr, help="Max instr")   

    parser.add_argument("--no-dump-wave", action='store_true', help="No dump wave")
    parser.add_argument("--wave-path", type=str, default=default_wave_path, help="Wave file")

    parser.add_argument("--fuzz-id", type=int, default=default_fuzz_id, help="Fuzz id")
    parser.add_argument("--run-snapshot", "-r", action='store_true', help="Run snapshot")

    parser.add_argument("--output-file", "-o", type=str, default=default_output_file, help="Output file")
    parser.add_argument("--err-file", type=str, default=default_err_file, help="Error file")

    args = parser.parse_args()

    if args.emu:
        run_emu(args)
    elif args.fuzz:
        run_fuzz(args)
    