#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FT 性能分析器 —— 双层监控 (系统级 + ROS 级)
================================================================================
独立脚本，不依赖 ROS2 也可运行基础系统监控。在 ROS2 环境中可同时监控话题频率。

用法:
  # 纯系统监控 (无 ROS)
  python3 src/scripts/perf_profiler.py --duration 30

  # ROS 环境系统 + 话题监控
  python3 src/scripts/perf_profiler.py --ros --topics /adc_rx/heartbeat --duration 60

  # 输出 JSON + Markdown 双报告
  python3 src/scripts/perf_profiler.py --ros --duration 60 -o perf_report

特性:
  ✓ 零 ROS 依赖也可运行 (仅监控系统指标)
  ✓ CPU 使用率、内存、swap、磁盘 I/O 实时采样
  ✓ ROS 话题频率监控 (需要 ros2 topic hz 命令)
  ✓ 自动生成 Markdown 报告 + JSON 原始数据
  ✓ 可配合 C++ FrameProfiler 使用 (分析节点内部分步耗时)

作者: zhengyuan.liu
日期: 2026-06-18
================================================================================
"""

import argparse
import json
import os
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime

# ============================================================================
# 系统指标采集器
# ============================================================================

class SystemMonitor:
    """从 /proc 文件系统采集系统级性能指标 (Linux/WSL 通用)"""

    def __init__(self):
        self.samples = []
        self._prev_cpu = None
        self._prev_io  = None
        self._boot_time = self._get_boot_time()

    # ── CPU ────────────────────────────────────────────────────────────────

    @staticmethod
    def _read_cpu_times():
        """读取 /proc/stat 的 CPU 时间 (聚合所有核心)"""
        with open('/proc/stat', 'r') as f:
            for line in f:
                if line.startswith('cpu '):
                    parts = line.split()
                    # cpu  user nice system idle iowait irq softirq steal guest guest_nice
                    vals = list(map(int, parts[1:9]))
                    return {
                        'user': vals[0], 'nice': vals[1], 'system': vals[2],
                        'idle': vals[3], 'iowait': vals[4] if len(vals) > 4 else 0,
                        'irq': vals[5] if len(vals) > 5 else 0,
                        'softirq': vals[6] if len(vals) > 6 else 0,
                        'steal': vals[7] if len(vals) > 7 else 0,
                    }
        return None

    def cpu_percent(self):
        """计算自上次采样以来的 CPU 使用率"""
        cur = self._read_cpu_times()
        if not cur or not self._prev_cpu:
            self._prev_cpu = cur
            return {}

        prev_total = sum(self._prev_cpu.values())
        cur_total  = sum(cur.values())
        total_delta = cur_total - prev_total
        if total_delta <= 0:
            self._prev_cpu = cur
            return {}

        idle_delta = cur['idle'] - self._prev_cpu['idle']
        iowait_delta = (cur.get('iowait', 0) - self._prev_cpu.get('iowait', 0))

        self._prev_cpu = cur
        return {
            'cpu_used_pct': round(100.0 * (total_delta - idle_delta) / total_delta, 1),
            'cpu_iowait_pct': round(100.0 * max(0, iowait_delta) / total_delta, 1),
        }

    # ── 内存 ────────────────────────────────────────────────────────────────

    @staticmethod
    def memory_info():
        """读取 /proc/meminfo 返回关键内存指标 (MB)"""
        info = {}
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if ':' in line:
                    key, val = line.split(':', 1)
                    val = val.strip().split()[0]  # 去掉 ' kB' 后缀
                    info[key.strip()] = int(val)
        return {
            'mem_total_mb':   round(info.get('MemTotal', 0) / 1024.0, 0),
            'mem_free_mb':    round(info.get('MemFree', 0) / 1024.0, 0),
            'mem_avail_mb':   round(info.get('MemAvailable', 0) / 1024.0, 0),
            'swap_total_mb':  round(info.get('SwapTotal', 0) / 1024.0, 0),
            'swap_free_mb':   round(info.get('SwapFree', 0) / 1024.0, 0),
            'swap_used_mb':   round((info.get('SwapTotal', 0) - info.get('SwapFree', 0)) / 1024.0, 0),
        }

    # ── 磁盘 I/O ────────────────────────────────────────────────────────────

    @staticmethod
    def _read_diskstats():
        """读取 /proc/diskstats 聚合所有磁盘"""
        total = {'read_sectors': 0, 'write_sectors': 0, 'read_ms': 0, 'write_ms': 0}
        try:
            with open('/proc/diskstats', 'r') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) < 14: continue
                    # 只统计物理磁盘 (sd*, nvme*, hd*)
                    dev = parts[2]
                    if not any(dev.startswith(p) for p in ('sd', 'nvme', 'hd', 'vd', 'xvd')):
                        continue
                    total['read_sectors']  += int(parts[5])
                    total['write_sectors'] += int(parts[9])
                    total['read_ms']       += int(parts[6])
                    total['write_ms']      += int(parts[10])
        except FileNotFoundError:
            pass
        return total

    def disk_io_rate(self):
        """计算磁盘 I/O 速率 (MB/s)"""
        cur = self._read_diskstats()
        if not self._prev_io:
            self._prev_io = cur
            return {}
        rd = (cur['read_sectors']  - self._prev_io['read_sectors'])  * 512 / 1048576.0
        wr = (cur['write_sectors'] - self._prev_io['write_sectors']) * 512 / 1048576.0
        self._prev_io = cur
        return {'disk_read_mb': round(rd, 2), 'disk_write_mb': round(wr, 2)}

    # ── 进程信息 ────────────────────────────────────────────────────────────

    @staticmethod
    def _get_boot_time():
        try:
            with open('/proc/stat', 'r') as f:
                for line in f:
                    if line.startswith('btime'):
                        return int(line.split()[1])
        except Exception:
            pass
        return 0

    @staticmethod
    def top_processes(n=5):
        """获取 CPU 占用前 N 的进程 (通过 ps 命令)"""
        try:
            result = subprocess.run(
                ['ps', '-eo', 'pid,pcpu,pmem,comm', '--sort=-pcpu', '--no-headers'],
                capture_output=True, text=True, timeout=5
            )
            lines = result.stdout.strip().split('\n')[:n]
            procs = []
            for line in lines:
                parts = line.split()
                if len(parts) >= 4:
                    procs.append({
                        'pid': int(parts[0]),
                        'cpu_pct': float(parts[1]),
                        'mem_pct': float(parts[2]),
                        'name': parts[3][:30]
                    })
            return procs
        except Exception:
            return []

    # ── 采样 ────────────────────────────────────────────────────────────────

    def sample(self):
        """采集一次全部系统指标"""
        cpu  = self.cpu_percent()
        mem  = self.memory_info()
        io   = self.disk_io_rate()
        proc = self.top_processes(5)
        ts   = time.time()

        sample = {
            'timestamp': ts,
            'elapsed_s': round(ts - self._boot_time, 1) if self._boot_time else 0,
            **cpu,
            **mem,
            **io,
            'top_cpu_procs': proc,
        }
        self.samples.append(sample)
        return sample


# ============================================================================
# ROS 话题监控器
# ============================================================================

class RosTopicMonitor:
    """通过 ros2 topic hz 命令监控话题频率"""

    def __init__(self, topics, timeout=5.0):
        self.topics = topics
        self.timeout = timeout
        self.rates = defaultdict(list)

    def measure(self):
        """对每个话题测量一次频率 (阻塞, 每个话题约 timeout 秒)"""
        results = {}
        for topic in self.topics:
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'hz', topic, '--window', '10'],
                    capture_output=True, text=True,
                    timeout=self.timeout + 5
                )
                # 解析最后一行: "average rate: X.XXX"
                for line in result.stderr.split('\n') + result.stdout.split('\n'):
                    if 'average rate' in line:
                        hz = float(line.strip().split()[-1])
                        results[topic] = hz
                        self.rates[topic].append(hz)
                        break
                else:
                    results[topic] = None
            except (subprocess.TimeoutExpired, Exception) as e:
                results[topic] = None
                print(f"  [WARN] 话题 {topic} 测量失败: {e}", file=sys.stderr)
        return results


# ============================================================================
# 报告生成器
# ============================================================================

class ReportGenerator:
    """从采样数据生成 Markdown + JSON 报告"""

    @staticmethod
    def markdown(system_samples, topic_rates, args):
        lines = []
        lines.append("# FT 性能分析报告")
        lines.append(f"\n**生成时间:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"**采样时长:** {args.duration}s")
        lines.append(f"**采样间隔:** {args.interval}s")
        lines.append(f"**环境:** {'ROS2' if args.ros else '纯系统'}")
        lines.append(f"**工作目录:** {os.getcwd()}")

        # ── 系统概览 ──
        lines.append("\n## 1. 系统资源概览\n")
        if system_samples:
            cpu_vals  = [s.get('cpu_used_pct', 0) for s in system_samples if 'cpu_used_pct' in s]
            io_wait   = [s.get('cpu_iowait_pct', 0) for s in system_samples if 'cpu_iowait_pct' in s]
            mem_avail = [s.get('mem_avail_mb', 0) for s in system_samples]
            swap_used = [s.get('swap_used_mb', 0) for s in system_samples]
            disk_r    = [s.get('disk_read_mb', 0) for s in system_samples if 'disk_read_mb' in s]
            disk_w    = [s.get('disk_write_mb', 0) for s in system_samples if 'disk_write_mb' in s]

            def stats(vals):
                if not vals: return ('N/A', 'N/A', 'N/A')
                return (f"{sum(vals)/len(vals):.1f}", f"{min(vals):.1f}", f"{max(vals):.1f}")

            lines.append("| 指标 | 均值 | 最小值 | 最大值 | 状态 |")
            lines.append("|------|------|--------|--------|------|")
            avg, vmin, vmax = stats(cpu_vals)
            warn = "⚠️ 高负载" if float(avg.replace('N/A','0')) > 80 else "✅ 正常"
            lines.append(f"| CPU 使用率 (%) | {avg} | {vmin} | {vmax} | {warn} |")
            avg, vmin, vmax = stats(io_wait)
            warn = "⚠️ IO 等待" if float(avg.replace('N/A','0')) > 20 else "✅ 正常"
            lines.append(f"| IO 等待 (%) | {avg} | {vmin} | {vmax} | {warn} |")
            avg, vmin, vmax = stats(mem_avail)
            warn = "⚠️ 内存不足" if float(avg.replace('N/A','999')) < 500 else "✅ 正常"
            lines.append(f"| 可用内存 (MB) | {avg} | {vmin} | {vmax} | {warn} |")
            avg, vmin, vmax = stats(swap_used)
            warn = "⚠️ Swap 使用中" if float(avg.replace('N/A','0')) > 100 else "✅ 正常"
            lines.append(f"| Swap 使用 (MB) | {avg} | {vmin} | {vmax} | {warn} |")
            if disk_r:
                avg_r, _, mx_r = stats(disk_r)
                avg_w, _, mx_w = stats(disk_w)
                lines.append(f"| 磁盘读取 (MB) | {avg_r} | — | {mx_r} | — |")
                lines.append(f"| 磁盘写入 (MB) | {avg_w} | — | {mx_w} | — |")

        # ── WSL 专项 ──
        lines.append("\n## 2. WSL / 环境诊断\n")
        wsl_issues = []

        # 检测是否在 WSL
        is_wsl = False
        try:
            with open('/proc/version', 'r') as f:
                ver = f.read().lower()
                is_wsl = 'microsoft' in ver or 'wsl' in ver
        except Exception:
            pass

        if is_wsl:
            lines.append("**检测到 WSL 环境** — 以下检查针对 WSL 特性:\n")
            lines.append("| 检查项 | 结果 | 建议 |")
            lines.append("|--------|------|------|")

            # 检查工作目录是否在 /mnt/ (Windows 文件系统)
            cwd = os.getcwd()
            on_drvfs = cwd.startswith('/mnt/')
            lines.append(f"| 工作目录在 drvfs? | {'⚠️ 是' if on_drvfs else '✅ 否'} | "
                         f"{'建议 cp 到 ~/ ' if on_drvfs else '—'} |")

            # Swap 使用
            if swap_used:
                avg_swap = sum(swap_used) / len(swap_used)
                lines.append(f"| Swap 活跃? | {'⚠️ 是 ({:.0f} MB)'.format(avg_swap) if avg_swap > 50 else '✅ 否'} | "
                             f"{'编辑 .wslconfig 限制内存' if avg_swap > 50 else '—'} |")

            # 可用内存
            if mem_avail:
                avg_mem = sum(mem_avail) / len(mem_avail)
                lines.append(f"| 可用内存 | {'⚠️ {:.0f} MB'.format(avg_mem) if avg_mem < 2048 else '✅ {:.0f} MB'.format(avg_mem)} | "
                             f"{'考虑增加 WSL 内存限制' if avg_mem < 2048 else '—'} |")
        else:
            lines.append("原生 Linux 环境 (非 WSL)。\n")

        # ── ROS 话题频率 ──
        if topic_rates:
            lines.append("\n## 3. ROS 话题频率\n")
            lines.append("| 话题 | 实测频率 (Hz) | 状态 |")
            lines.append("|------|---------------|------|")
            for topic, rates in topic_rates.items():
                last = rates[-1] if rates else 'N/A'
                if isinstance(last, (int, float)):
                    status = "✅ 正常" if last > 0.5 else "❌ 极低"
                    lines.append(f"| `{topic}` | {last:.2f} | {status} |")
                else:
                    lines.append(f"| `{topic}` | N/A | ❌ 测量失败 |")

        # ── TOP CPU 进程 ──
        if system_samples:
            last_sample = system_samples[-1]
            if last_sample.get('top_cpu_procs'):
                lines.append("\n## 4. TOP CPU 进程 (最后采样)\n")
                lines.append("| PID | CPU% | MEM% | 进程名 |")
                lines.append("|-----|------|------|--------|")
                for p in last_sample['top_cpu_procs']:
                    lines.append(f"| {p['pid']} | {p['cpu_pct']:.1f} | {p['mem_pct']:.1f} | {p['name']} |")

        # ── 诊断结论 ──
        lines.append("\n## 5. 诊断建议\n")
        suggestions = []

        if cpu_vals:
            avg_cpu = sum(cpu_vals) / len(cpu_vals)
            if avg_cpu > 80:
                suggestions.append("⚠️ **CPU 高负载**: 检查 `top` 确认哪个进程占用 CPU")
            if io_wait:
                avg_iow = sum(io_wait) / len(io_wait)
                if avg_iow > 10:
                    suggestions.append("⚠️ **IO 等待高**: 磁盘成为瓶颈，考虑减少文件 I/O 或预加载到内存")

        if swap_used:
            avg_sw = sum(swap_used) / len(swap_used)
            if avg_sw > 100:
                suggestions.append(f"❌ **Swap 活跃 ({avg_sw:.0f} MB)**: 内存不足，系统在换页！")
                suggestions.append("   → 减少预加载文件数 (`max_preload_files`)")
                suggestions.append("   → 检查 WSL `.wslconfig` 内存限制")

        if is_wsl and on_drvfs:
            suggestions.append("⚠️ **WSL drvfs 瓶颈**: 工作目录在 Windows 文件系统")
            suggestions.append("   → `cp -r /mnt/e/.../data ~/ft_data` 复制到 Linux 原生 fs")

        if topic_rates:
            for topic, rates in topic_rates.items():
                last = rates[-1] if rates else None
                if last and last < 1.0:
                    suggestions.append(f"❌ **{topic} 频率极低 ({last:.2f} Hz)**")
                    suggestions.append("   → 先确认测量工具自身不是瓶颈 (ros2 topic hz 在 Python 端反序列化 32MB 消息)")
                    suggestions.append("   → 查看 C++ 节点内部 FrameProfiler 日志确认内部发布频率")
                    suggestions.append("   → 检查 QoS: 大消息用 best_effort + keep_last(1) 避免背压")

        if not suggestions:
            suggestions.append("✅ 未检测到明显问题。若仍有疑问，请查看原始 JSON 数据。")

        for s in suggestions:
            lines.append(f"- {s}")

        lines.append(f"\n---\n*报告由 FT perf_profiler.py 自动生成*")
        return '\n'.join(lines)

    @staticmethod
    def json_report(system_samples, topic_rates, args):
        return json.dumps({
            'timestamp': datetime.now().isoformat(),
            'config': {
                'duration': args.duration,
                'interval': args.interval,
                'ros_enabled': args.ros,
                'topics': args.topics,
            },
            'system_samples': system_samples,
            'topic_rates': {k: v for k, v in topic_rates.items()},
        }, indent=2, ensure_ascii=False)


# ============================================================================
# 主程序
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='FT 性能分析器 — 系统级 + ROS 级双层监控',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s --duration 30                          纯系统监控 30 秒
  %(prog)s --ros -t /adc_rx/heartbeat -d 60       ROS + 系统监控 60 秒
  %(prog)s --ros -t /adc_rx/heartbeat -t /camera/image_raw -d 120 -o my_report
        """)
    parser.add_argument('-d', '--duration', type=int, default=30,
                        help='监控时长 (秒), 默认 30')
    parser.add_argument('-i', '--interval', type=float, default=1.0,
                        help='采样间隔 (秒), 默认 1.0')
    parser.add_argument('--ros', action='store_true',
                        help='启用 ROS 话题监控 (需要 ros2 命令)')
    parser.add_argument('-t', '--topics', action='append', default=[],
                        help='ROS 话题 (可重复使用); 默认监控 /adc_rx/heartbeat')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='报告文件路径前缀 (默认脚本同目录/perf_report)')

    args = parser.parse_args()

    if args.ros and not args.topics:
        args.topics = ['/adc_rx/heartbeat', '/adc/raw_data']

    print("=" * 60)
    print("  FT 性能分析器")
    print(f"  时长: {args.duration}s  间隔: {args.interval}s")
    print(f"  ROS 监控: {'开启' if args.ros else '关闭 (纯系统)'}")
    if args.topics:
        print(f"  话题: {', '.join(args.topics)}")
    print("=" * 60)
    print()

    # ── 系统监控 ──
    sys_mon = SystemMonitor()
    topic_mon = RosTopicMonitor(args.topics) if (args.ros and args.topics) else None

    start_time = time.time()
    iteration = 0

    print("采样中... (Ctrl+C 提前结束)\n")

    try:
        while time.time() - start_time < args.duration:
            iteration += 1
            loop_start = time.time()

            # 系统采样
            sample = sys_mon.sample()
            cpu_str = f"CPU:{sample.get('cpu_used_pct', '?'):>5}%"
            mem_str = f"MEM avail:{sample.get('mem_avail_mb', '?'):>6.0f}MB"
            swap_str = f"SWAP used:{sample.get('swap_used_mb', '?'):>6.0f}MB"
            io_str = ""
            if 'disk_read_mb' in sample:
                io_str = f" IO r:{sample['disk_read_mb']:.1f} w:{sample['disk_write_mb']:.1f} MB"
            print(f"  [{iteration:3d}] {cpu_str}  {mem_str}  {swap_str}{io_str}")

            # 话题采样 (每 10 轮测一次, ros2 topic hz 较慢)
            if topic_mon and iteration % 10 == 0:
                rates = topic_mon.measure()
                for t, hz in rates.items():
                    if hz:
                        print(f"         📡 {t}: {hz:.2f} Hz")

            # 休眠补偿
            elapsed = time.time() - loop_start
            sleep_time = max(0, args.interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n⚠️  用户中断")

    elapsed_total = time.time() - start_time
    print(f"\n采样完成: {len(sys_mon.samples)} 个样本, 实际耗时 {elapsed_total:.1f}s")

    # ── 话题最终测量 ──
    topic_rates = {}
    if topic_mon:
        print("\n最终话题频率测量...")
        topic_rates = topic_mon.measure()
        for t, hz in topic_rates.items():
            print(f"  {t}: {hz:.2f} Hz" if hz else f"  {t}: 测量失败")

    # ── 生成报告 ──
    reporter = ReportGenerator()

    base = args.output or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'perf_report')
    md_path   = base + '.md'
    json_path = base + '.json'

    with open(md_path, 'w', encoding='utf-8') as f:
        f.write(reporter.markdown(sys_mon.samples, topic_rates, args))

    with open(json_path, 'w', encoding='utf-8') as f:
        f.write(reporter.json_report(sys_mon.samples, topic_rates, args))

    print(f"\n{'=' * 60}")
    print(f"  报告已生成:")
    print(f"    Markdown: {os.path.abspath(md_path)}")
    print(f"    JSON:     {os.path.abspath(json_path)}")
    print(f"{'=' * 60}")


if __name__ == '__main__':
    main()
