#!/usr/bin/env python3
"""
Automated compatibility patcher for Ubuntu 24.04 (Noble raspi) + PREEMPT-RT.
Fixes:
  1. include/linux/rwsem.h: rw_base_assert_held_write(&sem->rwbase) pointer fix
  2. include/linux/rwbase_rt.h: missing #include <linux/bug.h> for WARN_ON
  3. include/linux/serial_core.h: missing nbcon_driver_acquire / release declarations
"""
import os
import sys

def patch_kernel(kernel_dir):
    print(f"Applying RT compatibility patches to kernel directory: {kernel_dir}")

    # 1. Fix rwsem.h pointer type mismatch
    rwsem_path = os.path.join(kernel_dir, "include/linux/rwsem.h")
    if os.path.exists(rwsem_path):
        with open(rwsem_path, "r") as f:
            c = f.read()
        if "rw_base_assert_held_write(sem)" in c:
            c = c.replace("rw_base_assert_held_write(sem)", "rw_base_assert_held_write(&sem->rwbase)")
            with open(rwsem_path, "w") as f:
                f.write(c)
            print("  [✓] Patched include/linux/rwsem.h")

    # 2. Fix rwbase_rt.h missing <linux/bug.h>
    rwbase_path = os.path.join(kernel_dir, "include/linux/rwbase_rt.h")
    if os.path.exists(rwbase_path):
        with open(rwbase_path, "r") as f:
            c = f.read()
        if "#include <linux/bug.h>" not in c and "#include <linux/atomic.h>" in c:
            c = c.replace("#include <linux/atomic.h>", "#include <linux/atomic.h>\n#include <linux/bug.h>")
            with open(rwbase_path, "w") as f:
                f.write(c)
            print("  [✓] Patched include/linux/rwbase_rt.h")

    # 3. Fix serial_core.h missing nbcon_driver_acquire / release declarations
    serial_core_path = os.path.join(kernel_dir, "include/linux/serial_core.h")
    if os.path.exists(serial_core_path):
        with open(serial_core_path, "r") as f:
            c = f.read()
        target = "static inline void __uart_port_nbcon_acquire"
        decl = (
            "struct console;\n"
            "#ifdef CONFIG_PRINTK\n"
            "extern void nbcon_driver_acquire(struct console *con);\n"
            "extern void nbcon_driver_release(struct console *con);\n"
            "#else\n"
            "static inline void nbcon_driver_acquire(struct console *con) {}\n"
            "static inline void nbcon_driver_release(struct console *con) {}\n"
            "#endif\n\n"
        )
        if target in c and "extern void nbcon_driver_acquire" not in c:
            c = c.replace(target, decl + target)
            with open(serial_core_path, "w") as f:
                f.write(c)
            print("  [✓] Patched include/linux/serial_core.h")

    # 4. Fix kernel/sched/fair.c update_deadline argument count mismatch
    fair_path = os.path.join(kernel_dir, "kernel/sched/fair.c")
    if os.path.exists(fair_path):
        with open(fair_path, "r") as f:
            c = f.read()
        import re
        c_new = re.sub(r'update_deadline\(\s*cfs_rq\s*,\s*curr\s*\);', 'update_deadline(cfs_rq, curr, tick);', c)
        c_new = re.sub(r'update_deadline\(\s*cfs_rq\s*,\s*se\s*\);', 'update_deadline(cfs_rq, se, false);', c_new)
        if c_new != c:
            with open(fair_path, "w") as f:
                f.write(c_new)
            print("  [✓] Patched kernel/sched/fair.c")

    # 5. Fix kernel/printk/printk.c suppress_panic_printk undeclared
    printk_path = os.path.join(kernel_dir, "kernel/printk/printk.c")
    if os.path.exists(printk_path):
        with open(printk_path, "r") as f:
            c = f.read()
        modified = False
        import re
        c_new = re.sub(
            r'if\s*\(\s*unlikely\(\s*suppress_panic_printk\s*\)\s*&&\s*other_cpu_in_panic\(\)\s*\)',
            'if (other_cpu_in_panic())',
            c
        )
        if c_new != c:
            c = c_new
            modified = True
        if "suppress_panic_printk" in c and "static int __read_mostly suppress_panic_printk" not in c:
            c = "static int __read_mostly suppress_panic_printk __maybe_unused;\n" + c
            modified = True
        if "void printk_legacy_allow_panic_sync" not in c:
            defn = (
                "\nstatic bool legacy_allow_panic_sync;\n\n"
                "void printk_legacy_allow_panic_sync(void)\n"
                "{\n"
                "\tlegacy_allow_panic_sync = true;\n\n"
                "\tif (printing_via_unlock && !in_nmi()) {\n"
                "\t\tif (console_trylock())\n"
                "\t\t\tconsole_unlock();\n"
                "\t}\n"
                "}\n"
            )
            c += defn
            modified = True
        if modified:
            with open(printk_path, "w") as f:
                f.write(c)
            print("  [✓] Patched kernel/printk/printk.c")

    # 6. Fix kernel/printk/internal.h missing console_prepend_replay prototype
    internal_path = os.path.join(kernel_dir, "kernel/printk/internal.h")
    if os.path.exists(internal_path):
        with open(internal_path, "r") as f:
            c = f.read()
        if "console_prepend_replay" not in c:
            target = "void console_prepend_dropped(struct printk_message *pmsg, unsigned long dropped);"
            if target in c:
                c = c.replace(target, target + "\nvoid console_prepend_replay(struct printk_message *pmsg);")
            else:
                c += "\nstruct printk_message;\nvoid console_prepend_replay(struct printk_message *pmsg);\n"
            with open(internal_path, "w") as f:
                f.write(c)
            print("  [✓] Patched kernel/printk/internal.h")

    # 7. Fix include/linux/perf_event.h missing pending_sigtrap field
    perf_path = os.path.join(kernel_dir, "include/linux/perf_event.h")
    if os.path.exists(perf_path):
        with open(perf_path, "r") as f:
            c = f.read()
        if "pending_sigtrap;" not in c and "pending_disable;" in c:
            import re
            c = re.sub(
                r'(unsigned int\s+pending_disable;)',
                r'\1\n\tunsigned int\t\t\tpending_sigtrap;',
                c
            )
            with open(perf_path, "w") as f:
                f.write(c)
            print("  [✓] Patched include/linux/perf_event.h")

    # 8. Fix include/linux/printk.h missing printk_legacy_allow_panic_sync & nbcon declarations
    printk_h_path = os.path.join(kernel_dir, "include/linux/printk.h")
    if os.path.exists(printk_h_path):
        with open(printk_h_path, "r") as f:
            c = f.read()
        if "printk_legacy_allow_panic_sync" not in c:
            block = (
                "\n/* PREEMPT-RT printk declarations */\n"
                "struct console;\n"
                "#ifdef CONFIG_PRINTK\n"
                "void printk_legacy_allow_panic_sync(void);\n"
                "extern void nbcon_driver_acquire(struct console *con);\n"
                "extern void nbcon_driver_release(struct console *con);\n"
                "void nbcon_atomic_flush_unsafe(void);\n"
                "#else\n"
                "static inline void printk_legacy_allow_panic_sync(void) {}\n"
                "static inline void nbcon_driver_acquire(struct console *con) {}\n"
                "static inline void nbcon_driver_release(struct console *con) {}\n"
                "static inline void nbcon_atomic_flush_unsafe(void) {}\n"
                "#endif\n"
                "bool this_cpu_in_panic(void);\n"
            )
            c += block
            with open(printk_h_path, "w") as f:
                f.write(c)
            print("  [✓] Patched include/linux/printk.h")

    # 9. Fix include/linux/sched.h missing task_is_pi_boosted
    sched_h_path = os.path.join(kernel_dir, "include/linux/sched.h")
    if os.path.exists(sched_h_path):
        with open(sched_h_path, "r") as f:
            c = f.read()
        if "task_is_pi_boosted" not in c:
            c += "\nextern bool task_is_pi_boosted(const struct task_struct *p);\n"
            with open(sched_h_path, "w") as f:
                f.write(c)
            print("  [✓] Patched include/linux/sched.h")

if __name__ == "__main__":
    kdir = sys.argv[1] if len(sys.argv) > 1 else "."
    patch_kernel(kdir)
