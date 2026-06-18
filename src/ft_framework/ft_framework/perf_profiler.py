#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FrameProfiler — 帧级处理环节耗时记录与评估工具
================================================================================
为 ROS2 节点提供轻量级 per-frame / per-step 计时，支持自动接入，周期性输出统计报告。

特性:
  - 零外部依赖，仅使用 Python 标准库
  - 三种 API 风格: context-manager / checkpoint / 装饰器
  - wrap_callback() 自动接管 tick/tick_end，无需手动调用
  - 每 N 帧自动输出 INFO 级格式化报告 + 可选落盘 JSON 报告
  - 分步统计: count / total / avg / min / max / last / std / 占比%
  - enabled=False 时所有计时调用退化为零开销
  - 自动检测 checkpoint 覆盖缺口（未计量时间）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
使用方式 A — 自动接入 + checkpoint (推荐，侵入最小)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  from ft_framework.perf_profiler import FrameProfiler

  class AdcRxNode(Node):
      def __init__(self):
          ...
          self._prof = FrameProfiler(self, log_every_n=50)
          self._prof.wrap_callback(self, '_on_timer')   # ← 一行接入
          self.timer = self.create_timer(period, self._on_timer)

      def _on_timer(self):
          sec, nsec = monotonic_us_stamp()
          self._prof.checkpoint('1.timestamp')

          data_array = np.random.randint(...)
          self._prof.checkpoint('2.np_random')

          msg.data = data_array.tobytes()
          self._prof.checkpoint('3.build_msg')

          self.pub_adc.publish(msg)
          self._prof.checkpoint('4.publish')


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
使用方式 B — 手动控制
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  def _on_timer(self):
      self._prof.tick()
      with self._prof.step('random'):
          data = np.random.randint(...)
      self._prof.tick_end()

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

作者: zhengyuan.liu
日期: 2026.6.17
================================================================================
"""

import functools
import json
import os
import time
from collections import defaultdict


# ============================================================================
# FrameProfiler
# ============================================================================

class FrameProfiler:
    """
    帧级分步计时器。

    参数:
      node:            ROS2 Node 实例 (用于 get_logger)
      log_every_n:     每隔多少帧输出一次性能报告 (默认 50)
      enabled:         是否启用计时 (False 时所有 API 退化为零开销)
      report_to_file:  是否同时将报告落盘为 JSON (默认 False)
      report_dir:      报告文件目录 (默认 ~/.ft_profiler/)
    """

    def __init__(self, node, log_every_n=50, enabled=True,
                 report_to_file=False, report_dir=None):
        self._node = node
        self._logger = node.get_logger()
        self._log_every_n = log_every_n
        self.enabled = enabled
        self._report_to_file = report_to_file
        self._report_dir = report_dir or os.path.join(
            os.path.expanduser('~'), '.ft_profiler')
        self._node_name = node.get_name()

        # 每步骤聚合统计
        self._steps = defaultdict(lambda: {
            'count': 0, 'total': 0.0, 'min': float('inf'), 'max': 0.0, 'last': 0.0,
        })
        # 每步骤原始耗时历史 (最近 history_max 条)
        self._step_history = defaultdict(list)
        self._history_max = log_every_n * 3

        # 帧级统计
        self._frame_count = 0
        self._frame_times = []
        self._frame_start = 0.0
        self._chkpt_time = 0.0
        self._step_order = []

        # 总运行统计 (不受 reset 影响)
        self._total_frames = 0
        self._total_time = 0.0

    # ==================================================================
    # 自动接入 API
    # ==================================================================

    def wrap_callback(self, obj, method_name: str):
        """
        自动接管实例方法的帧级计时。

        替换 obj.method_name: 入口自动 tick(), 出口自动 tick_end(), 异常安全。

        必须在 create_timer 之前调用。
        """
        original = getattr(obj, method_name)
        prof = self

        @functools.wraps(original)
        def wrapped(*args, **kwargs):
            prof.tick()
            try:
                return original(*args, **kwargs)
            finally:
                prof.tick_end()

        setattr(obj, method_name, wrapped)
        self._logger.debug(
            f'FrameProfiler 已接入 {obj.__class__.__name__}.{method_name}')
        return wrapped

    def wrap(self, func):
        """装饰器: 为独立函数添加帧级计时。"""
        prof = self

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            prof.tick()
            try:
                return func(*args, **kwargs)
            finally:
                prof.tick_end()

        return wrapper

    def checkpoint(self, name: str):
        """
        标记步骤分界点，记录自上一个 checkpoint (或 tick) 以来的耗时。

        用法:  代码块执行完毕后调用，名称描述刚执行完的步骤。
            do_step1()
            self._prof.checkpoint('1.step1')
            do_step2()
            self._prof.checkpoint('2.step2')
        """
        if not self.enabled:
            return
        now = time.perf_counter()
        ref = self._chkpt_time if self._chkpt_time > 0 else self._frame_start
        dt_ms = (now - ref) * 1000.0
        self._record_step(name, dt_ms)
        self._chkpt_time = now

    # ==================================================================
    # 手动控制 API
    # ==================================================================

    def tick(self):
        """标记新一帧开始。"""
        if not self.enabled:
            return
        now = time.perf_counter()
        self._frame_start = now
        self._chkpt_time = now

    def step(self, name: str):
        """返回 context manager，记录 with 块的执行耗时。"""
        return _StepContext(self, name)

    def tick_end(self):
        """标记帧结束，累计帧耗时，周期到达时自动输出报告。"""
        if not self.enabled:
            return
        now = time.perf_counter()
        frame_ms = (now - self._frame_start) * 1000.0
        self._frame_times.append(frame_ms)
        self._frame_count += 1
        self._total_frames += 1
        self._total_time += frame_ms

        # 捕获最后一个 checkpoint 到帧结束之间的未计量时间
        if self._chkpt_time > 0 and self._chkpt_time > self._frame_start:
            gap_ms = (now - self._chkpt_time) * 1000.0
            if gap_ms > 0.005:  # > 5μs 才记录
                self._record_step('_gap_', gap_ms)

        if self._frame_count % self._log_every_n == 0:
            self._print_report()
            if self._report_to_file:
                self._save_report_json()

    # ==================================================================
    # 报告输出
    # ==================================================================

    def report(self):
        """手动触发一次性能报告（无视周期计数器）。"""
        if not self.enabled or not self._frame_times:
            return
        self._print_report()

    def save_report(self, filepath: str = None):
        """手动保存当前统计为 JSON 文件。"""
        self._save_report_json(filepath)

    def reset(self):
        """重置当前统计窗口（_total_frames / _total_time 不清除）。"""
        self._steps.clear()
        self._step_history.clear()
        self._step_order.clear()
        self._frame_times.clear()
        self._frame_count = 0

    def finalize(self):
        """节点销毁前调用: 输出最终报告并落盘。"""
        if not self.enabled or not self._frame_times:
            return
        self._logger.info(
            f'FrameProfiler 最终统计: {self._total_frames} 帧, '
            f'总耗时 {self._total_time / 1000:.1f} s, '
            f'平均 {self.avg_frame_ms:.1f} ms/帧')
        self._print_report()
        if self._report_to_file:
            self._save_report_json()

    # ==================================================================
    # 属性
    # ==================================================================

    @property
    def avg_frame_ms(self) -> float:
        if not self._frame_times:
            return 0.0
        return sum(self._frame_times) / len(self._frame_times)

    @property
    def effective_hz(self) -> float:
        avg = self.avg_frame_ms
        return 1000.0 / avg if avg > 0 else 0.0

    @property
    def total_frames(self) -> int:
        return self._total_frames

    @property
    def report_dir(self) -> str:
        return self._report_dir

    def get_stats(self) -> dict:
        """返回当前统计快照，含原始历史供外部绘图分析。"""
        if not self._frame_times:
            return {}

        n = len(self._frame_times)
        total = sum(self._frame_times)
        avg = total / n

        steps_out = {}
        for name in self._step_order:
            s = self._steps[name]
            hist = self._step_history.get(name, [])
            steps_out[name] = {
                'count': s['count'],
                'total_ms': round(s['total'], 3),
                'avg_ms': round(s['total'] / s['count'], 3) if s['count'] else 0,
                'min_ms': round(s['min'], 3),
                'max_ms': round(s['max'], 3),
                'last_ms': round(s['last'], 3),
                'std_ms': round(_stddev(hist), 3) if len(hist) >= 2 else 0.0,
                'pct': round(s['total'] / total * 100, 1) if total else 0,
                'history_ms': [round(v, 3) for v in hist[-self._log_every_n:]],
            }

        return {
            'node': self._node_name,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
            'window_frames': n,
            'total_frames': self._total_frames,
            'frame_avg_ms': round(avg, 3),
            'frame_min_ms': round(min(self._frame_times), 3),
            'frame_max_ms': round(max(self._frame_times), 3),
            'frame_std_ms': round(_stddev(self._frame_times), 3),
            'effective_hz': round(self.effective_hz, 2),
            'frame_history_ms': [round(v, 3) for v in self._frame_times],
            'steps': steps_out,
        }

    # ==================================================================
    # 内部
    # ==================================================================

    def _record_step(self, name: str, dt_ms: float):
        s = self._steps[name]
        s['count'] += 1
        s['total'] += dt_ms
        s['last'] = dt_ms
        if dt_ms < s['min']:
            s['min'] = dt_ms
        if dt_ms > s['max']:
            s['max'] = dt_ms

        hist = self._step_history[name]
        hist.append(dt_ms)
        if len(hist) > self._history_max:
            hist.pop(0)

        if name not in self._step_order:
            self._step_order.append(name)

    @staticmethod
    def _fmt_ms(val: float) -> str:
        """智能格式化毫秒值: <0.01 显示为 '~0', 否则保留 1 位小数。"""
        if val < 0.005:
            return '    ~0'
        return f'{val:6.1f}'

    def _build_report_lines(self) -> list:
        if not self._frame_times:
            return ['(无数据)']

        n_frames = len(self._frame_times)
        total_frame = sum(self._frame_times)
        avg_frame = total_frame / n_frames
        min_frame = min(self._frame_times)
        max_frame = max(self._frame_times)
        eff_hz = 1000.0 / avg_frame if avg_frame > 0 else 0.0
        now_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())

        # 计算 step 覆盖的总时间
        step_sum = sum(s['total'] for s in self._steps.values())
        coverage = (step_sum / total_frame * 100.0) if total_frame > 0 else 100.0

        # ── 表头 ──
        lines = [
            '',
            '╔══════════════════════════════════════════════════════════════════════╗',
            f'║  {self._node_name} 性能报告  │  {now_str}  │  近 {n_frames} 帧 (总 #{self._total_frames})',
            '╠══════════════════════════════════════════════════════════════════════╣',
        ]

        # ── 分步统计 ──
        user_steps = [n for n in self._step_order if not n.startswith('_')]
        if user_steps:
            lines.append(
                f'║  {"步骤":<16s} {"avg":>7s}  {"min":>7s}  '
                f'{"max":>7s}  {"last":>7s}  {"占比":>6s} ║'
            )
            lines.append(
                f'║  {"─"*16}  {"─"*7}  {"─"*7}  {"─"*7}  {"─"*7}  {"─"*6} ║'
            )

        for name in user_steps:
            s = self._steps[name]
            avg = s['total'] / s['count'] if s['count'] else 0
            pct = (s['total'] / total_frame * 100.0) if total_frame > 0 else 0.0

            lines.append(
                f'║  {name:<16s} {self._fmt_ms(avg)}  {self._fmt_ms(s["min"])}  '
                f'{self._fmt_ms(s["max"])}  {self._fmt_ms(s["last"])}  {pct:5.1f}% ║'
            )

        # ── 未计量缺口 (如果有) ──
        gap_step = self._steps.get('_gap_')
        if gap_step and gap_step['count'] > 0:
            gap_avg = gap_step['total'] / gap_step['count']
            gap_pct = (gap_step['total'] / total_frame * 100.0) if total_frame > 0 else 0.0
            lines.append(
                f'║  {"(未计量/缺口)":<16s} {self._fmt_ms(gap_avg)}  '
                f'{self._fmt_ms(gap_step["min"])}  {self._fmt_ms(gap_step["max"])}  '
                f'{self._fmt_ms(gap_step["last"])}  {gap_pct:5.1f}% ║'
            )

        # ── 帧总耗时 ──
        lines.append('╠══════════════════════════════════════════════════════════════════════╣')
        lines.append(
            f'║  {"帧总耗时":<16s} {self._fmt_ms(avg_frame)}  '
            f'{self._fmt_ms(min_frame)}  {self._fmt_ms(max_frame)}  {"─":>7s}  {"100%":>6s} ║'
        )
        lines.append(
            f'║  等效: {eff_hz:.2f} Hz  │  目标: 15.00 Hz  │  '
            f'达成: {eff_hz / 15.0 * 100:.1f}%  │  覆盖率: {coverage:.0f}%'
        )

        # ── 诊断提示 ──
        if coverage < 90.0:
            lines.append(
                f'║  ⚠  checkpoint 覆盖率仅 {coverage:.0f}% — 存在大量未计量时间，请补全 checkpoint'
            )
        if eff_hz < 1.0:
            max_step = max(user_steps, key=lambda n: self._steps[n]['total'] / self._steps[n]['count']) if user_steps else None
            if max_step:
                s = self._steps[max_step]
                max_avg = s['total'] / s['count']
                max_pct = (s['total'] / total_frame * 100.0) if total_frame > 0 else 0
                lines.append(
                    f'║  → 最大瓶颈: "{max_step}" ({max_avg:.0f} ms, 占 {max_pct:.0f}%)'
                )

        lines.append('╚══════════════════════════════════════════════════════════════════════╝')
        return lines

    def _print_report(self):
        for line in self._build_report_lines():
            self._logger.info(line)

    def _save_report_json(self, filepath: str = None):
        stats = self.get_stats()
        if not stats:
            return

        if filepath is None:
            os.makedirs(self._report_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
            filepath = os.path.join(
                self._report_dir,
                f'profiler_{self._node_name}_{ts}.json')

        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(stats, f, indent=2, ensure_ascii=False)
            self._logger.info(f'性能报告已保存: {filepath}')
        except OSError as e:
            self._logger.warning(f'保存性能报告失败: {e}')


# ============================================================================
# _StepContext (内部 context manager)
# ============================================================================

class _StepContext:
    __slots__ = ('_profiler', '_name', '_t0')

    def __init__(self, profiler: FrameProfiler, name: str):
        self._profiler = profiler
        self._name = name
        self._t0 = 0.0

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        dt_ms = (time.perf_counter() - self._t0) * 1000.0
        self._profiler._record_step(self._name, dt_ms)
        return False


# ============================================================================
# 工具函数
# ============================================================================

def _stddev(values: list) -> float:
    n = len(values)
    if n < 2:
        return 0.0
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    return var ** 0.5
