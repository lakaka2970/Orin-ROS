#!/usr/bin/env python3
r"""
8T8R_dataset_gen.py  ---  FT_visualizer 数据 -> FT Radar Dataset 格式转换器

功能:
  将任意雷达点云 CSV + 航迹 CSV 转换为标准 FT Radar Dataset 格式。
  通过顶部的字段注册表 (FIELD_ALIASES) 适配不同格式的输入数据。

架构（三层解耦）:
  ┌─ Field Registry ─────────────────────┐
  │  字段别名表: 输入列名 → 标准字段名    │  ← 用户在此处配置
  ├─ Input Adapter ──────────────────────┤
  │  resolve_fields() 解析实际列名映射    │  ← 自动适配
  │  iter_frames() 按帧分组读取           │
  ├─ Data Transform ─────────────────────┤
  │  transform_points() 点云格式转换      │
  │  transform_tracks()  目标格式转换      │
  └─ Output Writer ──────────────────────┘
     write_dataset() 写入数据集文件

用法:
  python 8T8R_dataset_gen.py <点云CSV路径> [航迹CSV路径] [选项]

示例:
  python 8T8R_dataset_gen.py input_points.csv input_tracks.csv
  python 8T8R_dataset_gen.py input.csv -o /custom/output
"""

import argparse
import glob
import os
import sys
import numpy as np
import pandas as pd


# ═════════════════════════════════════════════════════════════════════
#  输入配置区 — 设置数据源路径
#  支持两种模式:
#    单文件模式: path 指向一个包含所有帧的 CSV
#    目录模式:   path 指向一个目录，每帧一个 CSV 文件（文件名 = 时间戳 μs）
#  设为 None 时通过命令行参数传入。
# ═════════════════════════════════════════════════════════════════════

# 点云数据源 — 同时支持文件或目录
POINT_CLOUD_SOURCE = None       # e.g. "data/points.csv" 或 "data/pc_csv/"
# 跟踪目标数据源 — 同时支持文件或目录
TRACK_SOURCE = None             # e.g. "data/tracks.csv" 或 "data/obj_csv/"

# ── 读取接口说明 ────────────────────────────────────────────────
# read_point_cloud(source, ...)   →  (DataFrame, field_map, ts_map)
# read_tracks(source, ...)        →  (DataFrame, field_map)
# 两个接口自动识别传入的是文件还是目录，并返回统一的标准化数据。
# 也可不传参，直接使用上面 POINT_CLOUD_SOURCE / TRACK_SOURCE 的值。
# ──────────────────────────────────────────────────────────────────


# ═════════════════════════════════════════════════════════════════════
#  Part 1 — 字段注册表 (Field Registry)
#  ════════════════════════════════════════════════════════════════════
#  修改/扩充下方字典即可适配不同格式的输入 CSV。
#  格式: {标准字段名: [别名1, 别名2, ...]}
#  解析时会按别名顺序匹配 CSV 的实际列名，找到即止。
# ═════════════════════════════════════════════════════════════════════

POINT_FIELD_ALIASES = {
    # ── 必需字段 ──
    'frame_id':        ['frameID', 'FrameID', 'frame_id', 'frame', 'Frame'],
    'x':               ['xpos', 'x_pos', 'xPos', 'XPos', 'x', 'X'],
    'y':               ['ypos', 'y_pos', 'yPos', 'YPos', 'y', 'Y'],
    'z':               ['zpos', 'z_pos', 'zPos', 'ZPos', 'z', 'Z'],
    # ── 可选: 径向距离 ──
    'range':           ['range', 'Range', 'dist', 'distance', 'Dist'],
    # ── 可选: 角度（单位自动探测: ° → rad）──
    'azimuth':         ['azimuthAng', 'azimuth', 'azAng', 'Azimuth', 'az'],
    'elevation':       ['eleAng', 'elevation', 'elAng', 'Elevation', 'el'],
    # ── 可选: 强度 ──
    'rcs':             ['rcsDb', 'RCS', 'rcs', 'rcs_dB', 'rcsDb'],
    'snr':             ['snrdB', 'SNR', 'snr', 'snr_dB', 'snrdB'],
    # ── 可选: 多普勒 ──
    'doppler':         ['radVelAbs', 'raw_doppler', 'doppler',
                        'radialVelocity', 'vel'],
    # ── 可选: 速度模糊 ──
    'ambvelocity_max': ['UnAmbvelocityMax', 'unambVelocityMax',
                        'unamb_vel_max'],
    'vel_amb_fac':     ['VelAmbFac', 'velAmbFac', 'ambigFactor'],
    'doppler_idx':     ['DopplerIDx', 'dopplerIdx', 'doppler_idx',
                        'Idx'],
    # ── 可选: 关联信息 ──
    'asso_trk_id':     ['assoTrkID', 'assoTrkId', 'AssoTrkID',
                        'trackId', 'track_id'],
    'doa_method':      ['DOAMethod', 'doaMethod', 'DOA', 'doa'],
}

TRACK_FIELD_ALIASES = {
    # ── 必需字段 ──
    'frame_id':    ['frameID', 'FrameID', 'frame_id', 'frame', 'Frame'],
    'obj_id':      ['objID', 'ObjID', 'obj_id', 'objectId',
                    'object_id', 'id'],
    'x':           ['objXPos', 'obj_x_pos', 'objX', 'xPos', 'XPos'],
    'y':           ['objYPos', 'obj_y_pos', 'objY', 'yPos', 'YPos'],
    # ── 可选: 速度 ──
    'vel_x':       ['objXVelAbs', 'obj_x_vel_abs', 'vx', 'Vx',
                    'velocity_x'],
    'vel_y':       ['objYVelAbs', 'obj_y_vel_abs', 'vy', 'Vy',
                    'velocity_y'],
    # ── 可选: 包围盒 ──
    'box_length':  ['objBoxLength', 'obj_box_length', 'length', 'Length', 'l'],
    'box_width':   ['objBoxWidth', 'obj_box_width', 'width', 'Width', 'w'],
    'box_height':  ['f32ObjBoxHeight', 'obj_box_height', 'height', 'Height', 'h'],
    # ── 可选: 朝向 ──
    'heading':     ['headingAngle', 'heading_angle', 'yaw', 'Yaw',
                    'heading'],
    # ── 可选: 质量 ──
    'confidence':  ['confidence', 'score', 'Score', 'conf',
                    'objConfidence'],
    'observ_hist': ['observHist', 'observ_hist', 'tracked_times',
                    'trackedTimes'],
    # ── 可选: 运动状态 ──
    'motion_pat':  ['objMotionPat', 'obj_motion_pat', 'motionPattern',
                    'motion_pattern'],
    'motion_dir':  ['eObjMotionDirect', 'motion_direct', 'motionDir',
                    'motion_direction'],
}

# ─── 字段分组说明（仅作文档，便于理解） ───
FIELD_GROUPS = {
    'point': {
        'required': ['frame_id', 'x', 'y', 'z'],
        'desc': '雷达点云 — 至少需要 帧ID + 三维坐标',
    },
    'track': {
        'required': ['frame_id', 'obj_id', 'x', 'y'],
        'desc': '雷达目标 — 至少需要 帧ID + 目标ID + 二维位置',
    },
}



def resolve_fields(columns, alias_table):
    """
    将 CSV 实际列名解析为标准字段映射。

    参数:
        columns: set[str] — CSV 中的实际列名
        alias_table: dict[str, list[str]] — 标准字段 → 别名列表

    返回:
        dict[str, str] — 标准字段名 → CSV 列名

    示例:
        >>> cols = {'xpos', 'ypos', 'zpos', 'frameID'}
        >>> resolve_fields(cols, POINT_FIELD_ALIASES)
        {'frame_id': 'frameID', 'x': 'xpos', 'y': 'ypos', 'z': 'zpos'}
    """
    mapping = {}
    for std_name, aliases in alias_table.items():
        for alias in aliases:
            if alias in columns:
                mapping[std_name] = alias
                break
    return mapping


def check_required(mapping, group, alias_table):
    """
    检查必需字段是否已解析，缺失则报错退出。
    """
    required = FIELD_GROUPS[group]['required']
    missing = [f for f in required if f not in mapping]
    if missing:
        print(f"[错误] {FIELD_GROUPS[group]['desc']}", file=sys.stderr)
        for f in missing:
            hint = f"  · 标准字段 '{f}' 的别名: {alias_table[f]}"
            print(hint, file=sys.stderr)
        sys.exit(1)


def detect_angle_unit(series):
    """
    探测角度列的单位: 'rad' 或 'deg'。
    如果 max(|values|) > π (3.14) 则判为度，否则判为弧度。
    """
    vals = series.dropna().values
    if len(vals) == 0:
        return 'rad'
    max_abs = np.nanmax(np.abs(vals.astype(float)))
    return 'deg' if max_abs > np.pi else 'rad'


def iter_frames(df, field_map):
    """
    按 frame_id 分组迭代数据，产出 (frame_id, group_df) 二元组。
    自动丢弃不在 timestamp_map 中的帧。
    """
    col = field_map['frame_id']
    for fid, group in df.groupby(col):
        yield fid, group


# ─── 常量（时间戳及字段默认值） ─────────────────────────────────
BASE_TIMESTAMP_US = 1083000000
DEFAULT_FRAME_INTERVAL_US = 50000   # 20 Hz
DEFAULT_AMBGT = 21.82
DEFAULT_EXIST_PROB = 100
DEFAULT_MULTI_TGT_PROB = 100
DEFAULT_AMBGT_PROB = 77
DEFAULT_IDX = 128
DEFAULT_SCORE = 0.85
DEFAULT_TRACKED_TIMES = 1
DEFAULT_MOVING_STATE = 0


def _build_timestamp_map(df, field_map,
                         base_us=BASE_TIMESTAMP_US,
                         interval_us=DEFAULT_FRAME_INTERVAL_US):
    """
    从 DataFrame 生成 {frame_id: timestamp_us} 映射。

    策略（自动）:
      1) 如果 DataFrame 有 'timStamp' 列，用其值（秒自动→微秒）
      2) 否则按 frameID 顺序线性生成
    """
    fid_col = field_map['frame_id']
    unique = sorted(df[fid_col].unique())

    if 'timStamp' in df.columns:
        frame_ts = df.groupby(fid_col)['timStamp'].first().to_dict()
        s = frame_ts[unique[0]]
        if 1e12 < s < 1e16:
            return {f: int(frame_ts[f]) for f in unique}
        else:
            mn = min(frame_ts.values())
            return {f: int((frame_ts[f] - mn) * 1_000_000) + base_us
                    for f in unique}
    return {f: base_us + i * interval_us for i, f in enumerate(unique)}


# ─── 底层读取辅助 ────────────────────────────────────────────────

def _read_single_csv(filepath, alias_table, group_key='point'):
    """读取单 CSV 文件并解析字段映射。"""
    df = pd.read_csv(filepath, low_memory=False)
    field_map = resolve_fields(set(df.columns), alias_table)
    check_required(field_map, group_key, alias_table)
    return df, field_map


def _read_csv_dir(directory, alias_table, group_key='point',
                  base_ts=BASE_TIMESTAMP_US):
    """
    读取目录下所有 CSV（每帧一个文件），合并为一个 DataFrame。
    文件名（不含扩展名）须为数字时间戳 (μs)。

    目录模式下 frame_id 不由 CSV 列提供，而是由文件名索引合成。
    """
    csv_files = sorted(glob.glob(os.path.join(directory, '*.csv')))
    if not csv_files:
        print(f"[错误] 目录中无任何 CSV 文件: {directory}", file=sys.stderr)
        sys.exit(1)

    frames = []
    first_fm = None
    for i, fp in enumerate(csv_files):
        stem = os.path.splitext(os.path.basename(fp))[0]
        try:
            ts_us = int(stem)
        except ValueError:
            print(f"[错误] 文件名不是有效时间戳 (μs): {stem}", file=sys.stderr)
            sys.exit(1)

        df_f = pd.read_csv(fp, low_memory=False)
        fm = resolve_fields(set(df_f.columns), alias_table)

        # ── 目录模式: frame_id 从文件索引合成，不从 CSV 列查找 ──
        if 'frame_id' not in fm:
            fm['frame_id'] = '_synth_frame_id'
        df_f[fm['frame_id']] = i

        if first_fm is None:
            first_fm = fm
            # 在目录模式下，frame_id 从索引合成，但其他必需字段仍需存在
            required = [f for f in FIELD_GROUPS[group_key]['required']
                        if f != 'frame_id']
            missing = [f for f in required if f not in first_fm]
            if missing:
                print(f"[错误] 目录中 CSV 缺少必需字段: {missing}", file=sys.stderr)
                print(f"      检查目录: {directory}", file=sys.stderr)
                sys.exit(1)

        frames.append((i, ts_us, df_f))

    if not frames:
        return pd.DataFrame(), {}

    ts_map = {fid: ts for fid, ts, _ in frames}
    combined = pd.concat([f for _, _, f in frames], ignore_index=True)
    return combined, first_fm, ts_map


# ─── 公用读取接口 ────────────────────────────────────────────────

def read_point_cloud(source=None, base_ts=BASE_TIMESTAMP_US,
                     interval_us=DEFAULT_FRAME_INTERVAL_US):
    """
    统一点云数据读取接口。

    参数:
        source: 文件路径 | 目录路径 | None
                为 None 时使用 POINT_CLOUD_SOURCE 的值。
        base_ts, interval_us: 单文件模式下生成时间戳的参数。

    返回:
        (DataFrame, field_map, {frame_id: timestamp_us})

    自动识别:
      · 文件 → 单 CSV 含多帧（需有 frameID 列）
      · 目录 → 每帧一个 CSV（文件名=时间戳 μs）
    """
    if source is None:
        source = POINT_CLOUD_SOURCE
    if source is None:
        print("[错误] 未指定点云数据源\n"
              "  请设置 POINT_CLOUD_SOURCE，或通过命令行参数传入",
              file=sys.stderr)
        sys.exit(1)

    if os.path.isfile(source):
        df, fm = _read_single_csv(source, POINT_FIELD_ALIASES, 'point')
        ts_map = _build_timestamp_map(df, fm, base_ts, interval_us)
        print(f"      [单文件模式] 字段: {', '.join(f'{k}={v}' for k, v in fm.items())}")
        return df, fm, ts_map

    if os.path.isdir(source):
        df, fm, ts_map = _read_csv_dir(source, POINT_FIELD_ALIASES, 'point',
                                       base_ts)
        print(f"      [目录模式   ] {len(ts_map)} 帧, "
              f"字段: {', '.join(f'{k}={v}' for k, v in fm.items())}")
        return df, fm, ts_map

    print(f"[错误] 路径不存在: {source}", file=sys.stderr)
    sys.exit(1)


def read_tracks(source=None):
    """
    统一跟踪目标数据读取接口。

    参数:
        source: 文件路径 | 目录路径 | None
                为 None 时使用 TRACK_SOURCE 的值。

    返回:
        (DataFrame, field_map)
    """
    if source is None:
        source = TRACK_SOURCE
    if source is None:
        print("[错误] 未指定航迹数据源\n"
              "  请设置 TRACK_SOURCE，或通过命令行参数传入",
              file=sys.stderr)
        sys.exit(1)

    if os.path.isfile(source):
        df, fm = _read_single_csv(source, TRACK_FIELD_ALIASES, 'track')
        print(f"      [单文件模式] 字段: {', '.join(f'{k}={v}' for k, v in fm.items())}")
        return df, fm

    if os.path.isdir(source):
        df, fm, _ = _read_csv_dir(source, TRACK_FIELD_ALIASES, 'track')
        print(f"      [目录模式   ] 字段: {', '.join(f'{k}={v}' for k, v in fm.items())}")
        return df, fm

    print(f"[错误] 路径不存在: {source}", file=sys.stderr)
    sys.exit(1)


# ═════════════════════════════════════════════════════════════════════
#  Part 3 — 数据转换器 (Data Transform)
#  ════════════════════════════════════════════════════════════════════

# ─── 常量 ───
# ─── 几何工具 ───

def _range(x, y, z):
    return float(np.sqrt(x*x + y*y + z*z))

def _azimuth(x, y):
    return float(np.arctan2(y, x))

def _elevation(z, xy_r):
    return float(np.arctan2(z, xy_r)) if xy_r > 1e-12 else 0.0

def transform_points(df, field_map, timestamp_map, output_dir, print_fn=print):
    """
    将点云 DataFrame 转换为逐帧数据集 CSV。

    参数:
        df:         原始点云 DataFrame
        field_map:  字段映射 {标准名: CSV列名}
        timestamp_map: {frame_id: timestamp_us}
        output_dir: 输出根目录
    """
    fm = field_map
    out_dir = os.path.join(output_dir, 'pc_csv_radar_front_center')
    os.makedirs(out_dir, exist_ok=True)

    has_range    = 'range' in fm
    has_azimuth  = 'azimuth' in fm
    has_el       = 'elevation' in fm
    has_rcs      = 'rcs' in fm
    has_snr      = 'snr' in fm
    has_doppler  = 'doppler' in fm
    has_ambvel   = 'ambvelocity_max' in fm
    has_ambfac   = 'vel_amb_fac' in fm
    has_dop_idx  = 'doppler_idx' in fm

    # ── 角度单位探测 ──
    az_unit = detect_angle_unit(df[fm['azimuth']]) if has_azimuth else 'rad'
    el_unit = detect_angle_unit(df[fm['elevation']]) if has_el else 'rad'
    az_scale = np.pi / 180.0 if az_unit == 'deg' else 1.0
    el_scale = np.pi / 180.0 if el_unit == 'deg' else 1.0
    if az_unit == 'deg':
        print_fn("  [提示] azimuth 列检测为角度单位(°), 自动转换为弧度")
    if el_unit == 'deg':
        print_fn("  [提示] elevation 列检测为角度单位(°), 自动转换为弧度")

    # ── 方位角可靠性检查: 标准差 < 0.5° 说明数据不可靠 ──
    az_reliable = True
    if has_azimuth and len(df) > 1:
        raw = df[fm['azimuth']].dropna().astype(float) * az_scale
        if raw.std() < np.deg2rad(0.5):
            az_reliable = False
            print_fn("  [提示] azimuth 列几乎恒定(标准差<0.5°), 使用计算值代替")

    out_cols = ['x','y','z','range','azimuth','elevation',
                'RCS','SNR','ambgt','exist_prob','multi_tgt_prob',
                'ambgt_prob','raw_doppler','idx']
    xc, yc, zc = fm['x'], fm['y'], fm['z']
    n = 0

    for fid, grp in iter_frames(df, fm):
        if fid not in timestamp_map:
            continue
        ts = timestamp_map[fid]
        rows = []
        for _, row in grp.iterrows():
            x = float(row[xc])
            y = float(row[yc])
            z = float(row[zc])

            r = _range(x, y, z)
            if has_range:
                rr = float(row[fm['range']])
                if rr > 0.1:
                    r = rr

            az = _azimuth(x, y)
            if has_azimuth and az_reliable:
                ra = float(row[fm['azimuth']]) * az_scale
                if -2*np.pi <= ra <= 2*np.pi:
                    az = ra

            el = _elevation(z, _range(x, y, 0))
            if has_el:
                re = float(row[fm['elevation']]) * el_scale
                if -2*np.pi <= re <= 2*np.pi and abs(re) <= 1.0:
                    el = re

            rcs = float(row[fm['rcs']]) if has_rcs else -20.0
            snr = float(row[fm['snr']]) if has_snr else 15.0

            ambgt = DEFAULT_AMBGT
            if has_ambvel:
                uv = float(row[fm['ambvelocity_max']])
                if uv > 0:
                    ambgt = round(uv * 2, 2)
            if has_ambfac:
                af = float(row[fm['vel_amb_fac']])
                if af > 0 and has_ambvel:
                    ambgt = round(float(row[fm['ambvelocity_max']]) / af, 2)

            dop = float(row[fm['doppler']]) if has_doppler else 0.0

            idx = DEFAULT_IDX
            if has_dop_idx:
                di = float(row[fm['doppler_idx']])
                idx = int(di) + 128 if di >= 0 else 192 - int(abs(di))

            rows.append([x, y, z, r, az, el, rcs, snr, ambgt,
                         DEFAULT_EXIST_PROB, DEFAULT_MULTI_TGT_PROB,
                         DEFAULT_AMBGT_PROB, dop, idx])

        pd.DataFrame(rows, columns=out_cols).to_csv(
            os.path.join(out_dir, f'{ts}.csv'), index=False,
            float_format='%.6f')
        n += 1
        print_fn(f"  [点云]  帧 {fid:>6}  -> {ts}.csv  ({len(rows):>4} 点)")
    print_fn(f"  [OK] 共写入 {n} 个点云文件")


def transform_tracks(df, field_map, timestamp_map, output_dir, print_fn=print):
    """
    将航迹 DataFrame 转换为逐帧目标 CSV。

    清洗规则（自动）:
      - obj_id == 0 的无效行
      - x == 0 and y == 0 的空行
    """
    fm = field_map
    out_dir = os.path.join(output_dir, 'obj_csv_radar')
    os.makedirs(out_dir, exist_ok=True)

    has_vx   = 'vel_x' in fm
    has_vy   = 'vel_y' in fm
    has_conf = 'confidence' in fm
    has_hist = 'observ_hist' in fm
    has_mpat = 'motion_pat' in fm
    has_mdir = 'motion_dir' in fm
    has_bl   = 'box_length' in fm
    has_bw   = 'box_width' in fm
    has_bh   = 'box_height' in fm
    has_yaw  = 'heading' in fm

    out_cols = ['object_id','tracked_times','score','x','y','z',
                'l','w','h','yaw','vx_absolute','vy_absolute',
                'vz_absolute','moving_state']
    idc = fm['obj_id']
    xc, yc = fm['x'], fm['y']
    n = 0

    for fid, grp in iter_frames(df, fm):
        if fid not in timestamp_map:
            continue
        ts = timestamp_map[fid]
        rows = []
        for _, row in grp.iterrows():
            oid = int(row[idc])
            if oid == 0:
                continue
            xp = float(row[xc])
            yp = float(row[yc])
            if abs(xp) < 1e-6 and abs(yp) < 1e-6:
                continue

            tracked = int(row[fm['observ_hist']]) if has_hist else DEFAULT_TRACKED_TIMES
            score = DEFAULT_SCORE
            if has_conf:
                c = float(row[fm['confidence']])
                if 0 < c <= 100:
                    score = round(c / 100.0, 4)

            vx = float(row[fm['vel_x']]) if has_vx else 0.0
            vy = float(row[fm['vel_y']]) if has_vy else 0.0
            bl = float(row[fm['box_length']]) if has_bl else 0.0
            bw = float(row[fm['box_width']]) if has_bw else 0.0
            bh = float(row[fm['box_height']]) if has_bh else 0.0
            yaw = float(row[fm['heading']]) if has_yaw else 0.0

            mstate = DEFAULT_MOVING_STATE
            if has_mpat:
                mp = int(row[fm['motion_pat']])
                mstate = {1: 1, 2: 2, 3: 3, 4: 0}.get(mp, 0)
            if has_mdir and mstate == DEFAULT_MOVING_STATE:
                if int(row[fm['motion_dir']]) == 0:
                    mstate = 1

            rows.append([oid, tracked, score, xp, yp, 0.0,
                         bl, bw, bh, yaw, vx, vy, 0.0, mstate])

        if rows:
            pd.DataFrame(rows, columns=out_cols).to_csv(
                os.path.join(out_dir, f'{ts}.csv'), index=False,
                float_format='%.6f')
            n += 1
            print_fn(f"  [目标]  帧 {fid:>6}  -> {ts}.csv  ({len(rows):>4} 目标)")
    print_fn(f"  [OK] 共写入 {n} 个目标文件")


# ═════════════════════════════════════════════════════════════════════
#  Part 4 — 输出写入器 (Output Writer)
#  ════════════════════════════════════════════════════════════════════

def write_ego_motion(timestamp_map, output_dir, print_fn=print):
    """生成 ego_motion.csv（模拟数据）。"""
    cols = ['timestamp_us','vx','yaw_rate','steering_angle','ax','ay','gear']
    rows = []
    for i, ts in enumerate(sorted(timestamp_map.values())):
        t = i * 0.05
        rows.append([ts,
                     round(10 + 2*np.sin(t*0.1), 6),
                     round(-0.05 + 0.02*np.cos(t*0.08), 6),
                     round((-0.05 + 0.02*np.cos(t*0.08)) * 15, 6),
                     round(0.5*np.cos(t*0.1), 7),
                     round(-0.3*np.sin(t*0.08), 7),
                     1])
    pd.DataFrame(rows, columns=cols).to_csv(
        os.path.join(output_dir, 'ego_motion.csv'), index=False,
        float_format='%.6f')
    print_fn(f"  [自车]  ego_motion.csv  ({len(rows)} 帧)")


def write_calibration(output_dir, print_fn=print):
    """生成示例标定 YAML。"""
    d = os.path.join(output_dir, 'calibration')
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'radar_front_center_ft.yaml'), 'w',
              encoding='utf-8') as f:
        f.write(r"""%%YAML:1.0
---
sensor_name: "radar_front_center"
sensor_type: ft_fvr60_eo
vehicle_xyz: front_left_up
r_s2b:
  [3.141563, 0.01370774, -1.034906e-09]
t_s2b:
  [4.052, -0.17, 0.677]
""")
    print_fn(f"  [标定]  {d}\\radar_front_center_ft.yaml")


# ═════════════════════════════════════════════════════════════════════
#  Part 5 — 主控流程
#  ════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description='雷达CSV数据 → FT Radar Dataset 格式转换器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('points_source', nargs='?', default=None,
                   help='点云源 — 文件路径 或 目录路径（默认: POINT_CLOUD_SOURCE）')
    p.add_argument('tracks_source', nargs='?', default=None,
                   help='航迹源 — 文件路径 或 目录路径（默认: TRACK_SOURCE）')
    p.add_argument('-o', '--output', default=None,
                   help='输出目录（默认: 脚本同目录的 dataset/）')
    p.add_argument('--base-ts', type=int, default=BASE_TIMESTAMP_US,
                   help=f'时间戳基准 μs（默认 {BASE_TIMESTAMP_US}）')
    p.add_argument('--interval', type=int,
                   default=DEFAULT_FRAME_INTERVAL_US,
                   help=f'帧间隔 μs（默认 {DEFAULT_FRAME_INTERVAL_US} = 20Hz）')
    return p.parse_args()


def main():
    args = parse_args()

    out_dir = os.path.abspath(args.output) if args.output else \
              os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'dataset')
    os.makedirs(out_dir, exist_ok=True)

    # 有效的数据源: CLI参数 > POINT_CLOUD_SOURCE/TRACK_SOURCE
    pts_src = args.points_source or POINT_CLOUD_SOURCE
    trk_src = args.tracks_source or TRACK_SOURCE

    def p(msg):
        print(msg)

    print("=" * 60)
    print("  8T8R 数据集生成工具  (字段注册表架构)")
    print("=" * 60)
    print(f"  点云源:    {pts_src or '(未设置)'}")
    print(f"  航迹源:    {trk_src or '(未设置)'}")
    print(f"  输出目录:  {out_dir}")
    print(f"  字段别名:  POINT_FIELD_ALIASES  ({len(POINT_FIELD_ALIASES)} 个标准字段)")
    print(f"             TRACK_FIELD_ALIASES  ({len(TRACK_FIELD_ALIASES)} 个标准字段)")
    print()

    # ── Step 1+2: 读取点云 + 时间戳（一步完成） ──
    print("[1/3] 读取点云数据 ...")
    df_pts, fm_pts, ts_map = read_point_cloud(
        pts_src, base_ts=args.base_ts, interval_us=args.interval)
    print(f"      -> {len(ts_map)} 个唯一帧")
    print()

    # ── Step 2: 转换点云 ──
    print("[2/3] 转换点云数据 ...")
    transform_points(df_pts, fm_pts, ts_map, out_dir, print_fn=p)
    print()

    # ── Step 3: 航迹 ──
    print("[3/3] 读取 + 转换航迹数据 ...")
    if trk_src:
        df_trk, fm_trk = read_tracks(trk_src)
        transform_tracks(df_trk, fm_trk, ts_map, out_dir, print_fn=p)
    else:
        print("  (跳过 — 未设置航迹源)")
    print()

    # ── 辅助文件 ──
    write_ego_motion(ts_map, out_dir, print_fn=p)
    write_calibration(out_dir, print_fn=p)
    print()

    # 汇总
    pc_d = os.path.join(out_dir, 'pc_csv_radar_front_center')
    obj_d = os.path.join(out_dir, 'obj_csv_radar')
    print("=" * 60)
    print("  完成！生成的文件:")
    print(f"    {out_dir}/")
    print(f"    ├── ego_motion.csv")
    print(f"    ├── calibration/")
    pc_n = len(os.listdir(pc_d)) if os.path.isdir(pc_d) else 0
    print(f"    ├── pc_csv_radar_front_center/  ({pc_n} 文件)")
    if trk_src:
        obj_n = len(os.listdir(obj_d)) if os.path.isdir(obj_d) else 0
        print(f"    └── obj_csv_radar/  ({obj_n} 文件)")
    print("=" * 60)


if __name__ == '__main__':
    main()
