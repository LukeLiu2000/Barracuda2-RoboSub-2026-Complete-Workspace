#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线系统辨识(训练对象 = 前向动力学模型的未知物理参数 θ,不是 MPPI)。

数据:training_data_collector_node 写出的 CSV
    t,experiment_id,13 维状态,8 维推力,下一时刻 13 维状态,dt
方法:阻尼高斯-牛顿(LM),纯 numpy 实现(服务器无需 scipy/torch)。
残差:单步预测误差 [Δp(3), Δangle(1), Δv(3), Δw(3)],各分量按尺度归一化;
损失:加权和 0.5||r||²;按 experiment_id 划分 Train/Val/Test;
验证:单步 + 10 步多步预测;导出:合并回 config/dynamics.yaml。

用法(在包目录内):
  python3 scripts/system_identification.py --data data/training_xxx.csv \
      --identify lin quad thrust --out config/dynamics.yaml
"""
import argparse
import hashlib
import re
import sys

import numpy as np

# ---------------------------------------------------------------------------
# 与 C++ DynamicsModel 完全一致的 numpy 复刻(改动需两边同步!)
# ---------------------------------------------------------------------------
NUM_THR = 8

DEFAULT_B_COLS = [
    ( 0.70711, 0.0,  0.70711, -0.0648,  0.5192,  0.0648),
    (-0.70711, 0.0,  0.70711, -0.0648, -0.5269, -0.0648),
    (-0.70711, 0.0, -0.70711,  0.0648,  0.5119, -0.0648),
    ( 0.70711, 0.0, -0.70711,  0.0648, -0.5041,  0.0648),
    (0.0, 1.0, 0.0,  0.1700,  0.0,    -0.3494),
    (0.0, 1.0, 0.0, -0.1955,  0.0,    -0.3494),
    (0.0, 1.0, 0.0, -0.1955,  0.0,     0.3416),
    (0.0, 1.0, 0.0,  0.1700,  0.0,     0.3416),
]

class Params:
    def __init__(self):
        self.mass = 40.0
        self.inertia = np.array([4.23, 5.60, 3.17])
        self.added_mass = np.array([12.0, 14.0, 10.0, 0.60, 0.80, 0.50])
        self.lin = np.array([15.0, 18.0, 15.0, 1.0, 1.2, 1.0])
        self.quad = np.array([45.0, 60.0, 60.0, 2.0, 3.0, 3.0])
        self.gravity_world = np.array([0.0, 0.0, -9.81])
        self.world_up = np.array([0.0, 0.0, 1.0])
        self.buoyancy = 387.4
        self.cb = np.array([0.0, 0.02, 0.0])
        self.thrust_scale = 1.0
        self.B = np.array(DEFAULT_B_COLS).T  # 6x8

def quat_mul(q1, q2):
    """Hamilton 积,输入输出 (x,y,z,w)"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*w2 + z1*x2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])

def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])

def step(p, q, v, w, u, dt, prm):
    """一步欧拉(与 C++ eulerStep 相同公式)。p/q/v/w 均为 numpy。"""
    u = np.clip(u, -39.2266, 39.2266)
    R = quat_to_R(q)
    nu = np.concatenate([v, w])
    tau = prm.B @ (prm.thrust_scale * u)
    # 恢复力
    f_g = R.T @ (prm.mass * prm.gravity_world)
    up = prm.world_up / np.linalg.norm(prm.world_up)
    f_b = R.T @ (prm.buoyancy * up)
    g6 = np.concatenate([f_g + f_b, np.cross(prm.cb, f_b)])
    # M
    M = np.diag(np.concatenate([
        np.full(3, prm.mass) + prm.added_mass[:3],
        prm.inertia + prm.added_mass[3:]]))
    # C = C_RB + C_A
    I_w = prm.inertia * w
    ma_v_w = prm.added_mass[:3] * v
    C = np.zeros((6, 6))
    C[:3, 3:] = -skew(prm.mass * v + ma_v_w)
    C[3:, :3] = -C[:3, 3:]
    C[3:, 3:] = -skew(I_w + prm.added_mass[3:] * w)
    # D
    d = prm.lin + prm.quad * np.abs(nu)
    rhs = tau + g6 - C @ nu - d * nu
    nu_dot = np.linalg.solve(M, rhs)
    v2 = v + nu_dot[:3] * dt
    w2 = w + nu_dot[3:] * dt
    p2 = p + (R @ v) * dt
    qd = quat_mul(q, np.array([w[0], w[1], w[2], 0.0]))  # 用步进前的 w(与 C++ 一致)
    q2 = q + 0.5 * qd * dt
    q2 /= np.linalg.norm(q2)
    return p2, q2, v2, w2

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]])

def angle_between(qa, qb):
    d = quat_mul(np.array([-qa[0], -qa[1], -qa[2], qa[3]]), qb)
    if d[3] < 0:
        d = -d
    return 2.0 * np.arctan2(np.linalg.norm(d[:3]), abs(d[3]))

# ---------------------------------------------------------------------------
# 数据
# ---------------------------------------------------------------------------
def load_csv(path):
    rows = []
    with open(path) as f:
        header = f.readline().strip().split(',')
        need = 1 + 1 + 13 + 8 + 13 + 1
        for line in f:
            parts = line.strip().split(',')
            if len(parts) != need:
                continue
            try:
                vals = [float(x) for x in parts[:1] + parts[2:]]
            except ValueError:
                continue
            rows.append((parts[1], np.array(vals)))
    if not rows:
        sys.exit('no data rows')
    arr = np.array([v for _, v in rows])
    exp_ids = [e for e, _ in rows]
    return header, arr, exp_ids
    # arr 列:0=t, 1..13=x, 14..21=u, 22..34=x_next, 35=dt

def split_by_exp(exp_ids, ratios=(0.7, 0.15, 0.15), salt='auv'):
    """按 experiment_id 哈希划分,保证同实验不跨集"""
    buckets = {'train': [], 'val': [], 'test': []}
    for i, e in enumerate(exp_ids):
        h = int(hashlib.md5((salt + e).encode()).hexdigest(), 16) / 2**128
        k = 'train' if h < ratios[0] else ('val' if h < ratios[0]+ratios[1] else 'test')
        buckets[k].append(i)
    return {k: np.array(v) for k, v in buckets.items()}

# ---------------------------------------------------------------------------
# 残差与 LM
# ---------------------------------------------------------------------------
SCALES = dict(p=0.05, ang=0.05, v=0.10, w=0.20)  # 各分量归一化尺度

def residuals_batch(theta, which, base, idx, arr):
    prm = params_from_theta(theta, which, base)
    # 向量化:逐样本循环(样本 ≤ 数万,纯 python 循环 + numpy 内核足够)
    res = np.empty((len(idx), 10))
    for j, i in enumerate(idx):
        t, x, u, xn, dt = unpack(arr, i)
        p2, q2, v2, w2 = step(x[:3], x[3:7], x[7:10], x[10:13], u, dt, prm)
        pn, qn, vn, wn = xn[:3], xn[3:7], xn[7:10], xn[10:13]
        res[j, 0:3] = (p2 - pn) / SCALES['p']
        res[j, 3] = angle_between(q2, qn) / SCALES['ang']
        res[j, 4:7] = (v2 - vn) / SCALES['v']
        res[j, 7:10] = (w2 - wn) / SCALES['w']
    return res

def unpack(arr, i):
    t = arr[i, 0]
    x = arr[i, 1:14]
    u = arr[i, 14:22]
    xn = arr[i, 22:35]
    dt = arr[i, 35]
    return t, x, u, xn, dt

def params_from_theta(theta, which, base):
    prm = Params()
    prm.mass, prm.inertia, prm.added_mass = base['mass'], base['inertia'].copy(), base['added_mass'].copy()
    prm.lin, prm.quad = base['lin'].copy(), base['quad'].copy()
    prm.gravity_world = base['gravity_world']
    prm.world_up = base['world_up']
    prm.buoyancy, prm.cb = base['buoyancy'], base['cb']
    prm.thrust_scale = base['thrust_scale']
    k = 0
    if 'added' in which:
        n = 6
        prm.added_mass = np.maximum(theta[k:k+n], 1e-3); k += n
    if 'lin' in which:
        n = 6
        prm.lin = np.maximum(theta[k:k+n], 0.0); k += n
    if 'quad' in which:
        n = 6
        prm.quad = np.maximum(theta[k:k+n], 0.0); k += n
    if 'thrust' in which:
        prm.thrust_scale = max(theta[k], 0.05); k += 1
    return prm

def theta_init(which, base):
    th = []
    if 'added' in which: th += list(base['added_mass'])
    if 'lin' in which: th += list(base['lin'])
    if 'quad' in which: th += list(base['quad'])
    if 'thrust' in which: th += [base['thrust_scale']]
    return np.array(th)

def finite_jacobian(theta, which, base, idx, arr, eps_scale=1e-4):
    J = np.empty((len(idx) * 10, len(theta)))
    for k in range(len(theta)):
        eps = max(abs(theta[k]) * eps_scale, 1e-4)
        tp = theta.copy(); tp[k] += eps
        tm = theta.copy(); tm[k] -= eps
        rp = residuals_batch(tp, which, base, idx, arr).ravel()
        rm = residuals_batch(tm, which, base, idx, arr).ravel()
        J[:, k] = (rp - rm) / (2 * eps)
    return J

def lm_optimize(theta0, which, base, idx, arr, iters=20, verbose=True):
    theta = theta0.copy()
    lam = 1e-2
    r = residuals_batch(theta, which, base, idx, arr)
    loss = 0.5 * float(r.ravel() @ r.ravel())
    for it in range(iters):
        J = finite_jacobian(theta, which, base, idx, arr)
        g = J.T @ r.ravel()
        H = J.T @ J
        ok = False
        for _ in range(8):  # 阻尼自适应
            try:
                dtheta = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-9), -g)
            except np.linalg.LinAlgError:
                lam *= 10; continue
            th2 = theta + dtheta
            r2 = residuals_batch(th2, which, base, idx, arr)
            l2 = 0.5 * float(r2.ravel() @ r2.ravel())
            if l2 < loss:
                theta, r, loss = th2, r2, l2
                lam = max(lam * 0.3, 1e-9)
                ok = True
                break
            lam *= 10
        if verbose:
            print(f'  iter {it:2d}  loss={loss:.4f}  lam={lam:.1e}')
        if not ok or (np.linalg.norm(dtheta) < 1e-7 * max(1, np.linalg.norm(theta))):
            break
    return theta, loss

def multistep_eval(theta, which, base, idx, arr, steps=10, n_points=30):
    prm = params_from_theta(theta, which, base)
    errs_v, errs_p = [], []
    idx = np.asarray(idx)
    if len(idx) < steps + 1:
        return float('nan'), float('nan')
    starts = np.linspace(0, len(idx) - steps - 1, min(n_points, len(idx) - steps - 1)).astype(int)
    for s0 in starts:
        i = idx[s0]
        t, x, u, xn, dt = unpack(arr, i)
        p, q, v, w = x[:3], x[3:7], x[7:10], x[10:13]
        for s in range(steps):
            j = idx[s0 + s]
            _, _, uj, _, _ = unpack(arr, j)
            p, q, v, w = step(p, q, v, w, uj, dt, prm)
        i_end = idx[s0 + steps]
        _, xn, _, _, _ = unpack(arr, i_end)
        errs_v.append(np.linalg.norm(v - xn[7:10]))
        errs_p.append(np.linalg.norm(p - xn[:3]))
    return float(np.mean(errs_p)), float(np.mean(errs_v))

# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------
def export_yaml(theta, which, base, out_path):
    prm = params_from_theta(theta, which, base)
    def fmt(v):
        return '[' + ', '.join(f'{x:.4f}' for x in v) + ']'
    text = open(out_path, encoding='utf-8').read()
    text = re.sub(r'(added_mass:\s*)\[([^\]]*)\]',
                  lambda m: m.group(1) + fmt(prm.added_mass), text)
    text = re.sub(r'(linear_damping:\s*)\[([^\]]*)\]',
                  lambda m: m.group(1) + fmt(prm.lin), text)
    text = re.sub(r'(quadratic_damping:\s*)\[([^\]]*)\]',
                  lambda m: m.group(1) + fmt(prm.quad), text)
    text = re.sub(r'(thrust_scale:\s*)[-0-9.eE+]+',
                  lambda m: m.group(1) + f'{prm.thrust_scale:.4f}', text)
    stamp = '# ==== system_identification.py 辨识后自动更新(原始值见 dynamics_default.yaml)====\n'
    if '辨识后自动更新' not in text:
        text = stamp + text
    open(out_path, 'w', encoding='utf-8').write(text)
    print(f'导出 -> {out_path}')

# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True, help='collector CSV 路径')
    ap.add_argument('--identify', nargs='+', default=['lin', 'quad', 'thrust'],
                    choices=['added', 'lin', 'quad', 'thrust'],
                    help='要辨识的参数组')
    ap.add_argument('--out', default='config/dynamics.yaml')
    ap.add_argument('--iters', type=int, default=20)
    ap.add_argument('--max-samples', type=int, default=20000,
                    help='训练用样本上限(LM 有限差分 Jacobian 成本考虑)')
    args = ap.parse_args()

    header, arr, exp_ids = load_csv(args.data)
    n = len(arr)
    print(f'数据:{n} 行,{len(set(exp_ids))} 个实验,字段数 {len(header)}')
    splits = split_by_exp(exp_ids)
    for k, v in splits.items():
        print(f'  {k}: {len(v)} 行')
    tr = splits['train']
    if len(tr) > args.max_samples:
        sel = np.random.default_rng(0).choice(tr, args.max_samples, replace=False)
        tr = np.sort(sel)
        print(f'  训练集下采样 -> {len(tr)} 行')

    # 基准参数(与 config/dynamics.yaml 默认一致;可先 --dry-run 看初值误差)
    base = dict(mass=40.0, inertia=np.array([4.23, 5.60, 3.17]),
                added_mass=np.array([12.0, 14.0, 10.0, 0.60, 0.80, 0.50]),
                lin=np.array([15.0, 18.0, 15.0, 1.0, 1.2, 1.0]),
                quad=np.array([45.0, 60.0, 60.0, 2.0, 3.0, 3.0]),
                gravity_world=np.array([0.0, 0.0, -9.81]),
                world_up=np.array([0.0, 0.0, 1.0]),
                buoyancy=387.4, cb=np.array([0.0, 0.02, 0.0]),
                thrust_scale=1.0)

    theta0 = theta_init(args.identify, base)
    print(f'辨识参数组 {args.identify},维度 {len(theta0)}')

    r0 = residuals_batch(theta0, args.identify, base, splits['test'], arr)
    print(f'初值 Test 单步 RMS 残差: {np.sqrt(np.mean(r0**2)):.4f}')

    theta, loss = lm_optimize(theta0, args.identify, base, tr, arr,
                              iters=args.iters)
    print(f'训练收敛 loss = {loss:.4f}')

    for k in ('train', 'val', 'test'):
        if len(splits[k]) == 0:
            continue
        r = residuals_batch(theta, args.identify, base, splits[k], arr)
        print(f'{k:5s} 单步 RMS: {np.sqrt(np.mean(r**2)):.4f}')

    if len(splits['test']) > 11:
        p_err, v_err = multistep_eval(theta, args.identify, base, splits['test'], arr)
        print(f'Test 10 步多步预测: 平均位置误差 {p_err:.4f} m,'
              f'平均速度误差 {v_err:.4f} m/s')

    prm = params_from_theta(theta, args.identify, base)
    print('\n辨识结果:')
    if 'added' in args.identify:
        print(f'  added_mass   = {prm.added_mass}')
    if 'lin' in args.identify:
        print(f'  linear_damping = {prm.lin}')
    if 'quad' in args.identify:
        print(f'  quadratic_damping = {prm.quad}')
    if 'thrust' in args.identify:
        print(f'  thrust_scale = {prm.thrust_scale:.4f}')

    export_yaml(theta, args.identify, base, args.out)

if __name__ == '__main__':
    main()
