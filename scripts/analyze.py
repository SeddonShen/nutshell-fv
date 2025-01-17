import os
import argparse

NOOP_HOME = os.getenv("NOOP_HOME")

class Snapshot:
    class RegInt:
        reg_name = ["zero", "ra", "sp", "gp",
                    "tp", "t0", "t1", "t2",
                    "s0", "s1", "a0", "a1",
                    "a2", "a3", "a4", "a5",
                    "a6", "a7", "s2", "s3",
                    "s4", "s5", "s6", "s7",
                    "s8", "s9", "s10", "s11",
                    "t3", "t4", "t5", "t6"]
        value = {}
        
        def __init__(self):
            for name in self.reg_name:
                self.value[name] = 0
    
    class RegFloat:
        reg_name = ["ft0", "ft1", "ft2", "ft3",
                    "ft4", "ft5", "ft6", "ft7",
                    "fs0", "fs1", "fa0", "fa1",
                    "fa2", "fa3", "fa4", "fa5",
                    "fa6", "fa7", "fs2", "fs3",
                    "fs4", "fs5", "fs6", "fs7",
                    "fs8", "fs9", "fs10", "fs11",
                    "ft8", "ft9", "ft10", "ft11"]
        value = {}

        def __init__(self):
            for name in self.reg_name:
                self.value[name] = 0
    
    class RegCSR:
        reg_name = ["privilegeMode", "mstatus", "sstatus", "mepc",
                    "sepc", "mtval", "stval", "mtvec",
                    "stvec", "mcause", "scause", "satp",
                    "mip", "mie", "mscratch", "sscratch",
                    "mideleg", "medeleg"]
        value = {}
        
        def __init__(self):
            for name in self.reg_name:
                self.value[name] = 0
    
    class CSR_Buffer:
        value = [0] * 4096
    
    cycleCnt = 0
    reg_int = RegInt()
    reg_fp = RegFloat()
    reg_csr = RegCSR()
    pc = 0
    csr_buffer = CSR_Buffer()

    def input_int_regs(self):
        pass
    
    def output_int_regs(self):
        print("===== Integer Registers =====")
        max_name_length = max(len(name) for name in self.reg_int.reg_name)
        reg_items = list(self.reg_int.value.items())
        for i in range(0, len(reg_items), 4):
            # Generate a row with up to four registers
            row = " ".join(f"{name:>{max_name_length}}({'x'+str(i+cnt):>3}): {value:016x}"
                            for cnt, (name, value) in enumerate(reg_items[i:i + 4]))
            print(row)
    
    def output_fp_regs(self):
        print("===== Floating Point Registers =====")
        max_name_length = max(len(name) for name in self.reg_fp.reg_name)
        reg_items = list(self.reg_fp.value.items())
        for i in range(0, len(reg_items), 4):
            # Generate a row with up to four registers
            row = " ".join(f"{name:>{max_name_length}}: {value:016x}"
                            for name, value in reg_items[i:i + 4])
            print(row)
    
    def output_csr_regs(self):
        print("===== CSR Registers =====")
        print("Cycle Count: ", self.cycleCnt)
        print("PC: 0x{:016x}".format(self.pc))
        # privilegeMode
        print("privilegeMode: ", self.reg_csr.value["privilegeMode"])
        # mstatus, mcause, mepc
        print("mstatus: 0x{:016x} mcause: 0x{:016x} mepc: 0x{:016x}".format(
            self.reg_csr.value["mstatus"], self.reg_csr.value["mcause"], self.reg_csr.value["mepc"]))
        # sstatus, scause, sepc
        print("sstatus: 0x{:016x} scause: 0x{:016x} sepc: 0x{:016x}".format(
            self.reg_csr.value["sstatus"], self.reg_csr.value["scause"], self.reg_csr.value["sepc"]))
        # satp
        print("satp: 0x{:016x}".format(self.reg_csr.value["satp"]))
        # mip, mie
        print("mip: 0x{:016x} mie: 0x{:016x}".format(self.reg_csr.value["mip"], self.reg_csr.value["mie"]))
        # mideleg, medeleg
        print("mideleg: 0x{:016x} medeleg: 0x{:016x}".format(
            self.reg_csr.value["mideleg"], self.reg_csr.value["medeleg"]))
        # mtval, stval, mtvec, stvec
        print("mtval: 0x{:016x} stval: 0x{:016x} mtvec: 0x{:016x} stvec: 0x{:016x}".format(
            self.reg_csr.value["mtval"], self.reg_csr.value["stval"], self.reg_csr.value["mtvec"], self.reg_csr.value["stvec"]))
        # mscratch, sscratch
        print("mscratch: 0x{:016x} sscratch: 0x{:016x}".format(
            self.reg_csr.value["mscratch"], self.reg_csr.value["sscratch"]))

def snapshot_parser(snapshot_id):
    snapshot_file = os.path.join(NOOP_HOME, "ccover", "SetInitValues", "csr_snapshot", f"{snapshot_id}")

    snapshot = Snapshot()

    with open(snapshot_file, 'rb') as f:
        # snapshot.cycleCnt = int.from_bytes(f.read(8), byteorder='little')
        for i in range(32):
            snapshot.reg_int.value[snapshot.reg_int.reg_name[i]] = int.from_bytes(f.read(8), byteorder='little')
        # for i in range(32):
        #     snapshot.reg_fp.value[snapshot.reg_fp.reg_name[i]] = int.from_bytes(f.read(8), byteorder='little')
        for i in range(18):
            snapshot.reg_csr.value[snapshot.reg_csr.reg_name[i]] = int.from_bytes(f.read(8), byteorder='little')
            # print(f"{snapshot.reg_csr.reg_name[i]}: {snapshot.reg_csr.value[snapshot.reg_csr.reg_name[i]]}")
        snapshot.pc = int.from_bytes(f.read(8), byteorder='little')
        # for i in range(4096):
        #     snapshot.csr_buffer.value[i] = int.from_bytes(f.read(8), byteorder='little')
    
    snapshot.output_int_regs()
    # snapshot.output_fp_regs()
    snapshot.output_csr_regs()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    
    parser.add_argument("--snapshot", "-s", type=int, help="Snapshot id")

    args = parser.parse_args()
    
    snapshot_parser(args.snapshot)