#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_full_report —— 全源码 PDF 报告生成器(Isaac python + reportlab)。

  /isaacsim/python.sh make_full_report.py --ws <工作区> --out <输出目录> \
      [--data report_data.json]

产出:
  RoboSub_Controller_System_Full_Source_Report_YYYY-MM-DD.pdf
  manifest.json(逐文件 SHA-256)
  render_check.json(结构化打开校验:%PDF 头/%%EOF/页对象计数)

报告结构:13 个内容章节(概述/结构/接口/推进器/PID/GRU/MPPI 保留/扰动/
可视化/实验系统/验证结果/限制声明)+ 附录A 全部包完整源码(逐文件路径头,
不遗漏文本文件;二进制如实标注跳过)+ 附录B 清单与校验和。
秘密脱敏:password/token/secret/api_key 赋值模式打码后再入 PDF。
系统 python 的 matplotlib/numpy 冲突,故统一用 Isaac python。
"""

import argparse
import datetime as _dt
import hashlib
import json
import os
import re
import sys

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, PageTemplate,
                                Paragraph, Preformatted, Spacer, Table,
                                TableStyle)
from reportlab.platypus.tableofcontents import TableOfContents

TEXT_EXT = (".py", ".cpp", ".hpp", ".h", ".c", ".json", ".yaml", ".yml",
            ".xml", ".txt", ".md", ".cfg", ".sh", ".launch.py", ".urdf",
            ".sdf", ".ps1", ".in", ".gitignore", ".repos", "")
SECRET_RE = re.compile(
    r"(?i)(password|passwd|secret|token|api[_-]?key|authorization)"
    r"(\s*[=:]\s*)([^\s\"'}\]]+)")


def mask_secrets(text):
    return SECRET_RE.sub(lambda m: m.group(1) + m.group(2) + "***MASKED***",
                         text)


def register_cjk():
    """注册 CJK 字体(正文与代码);返回 (body, mono)。

    约束:reportlab 的 TTFont 只支持 TrueType 轮廓(glyf)。本机
    NotoSansCJK-*.ttc 为 CFF(PostScript)轮廓,加载必抛
    "postscript outlines are not supported"(2026-11-08 实测);
    DroidSansFallbackFull.ttf 为 TrueType 轮廓且覆盖 CJK,改用之。
    本机无 CJK 等宽 TrueType 字体,代码字体退回与正文同一字体
    (非等宽,如实说明;源码以折行保证完整呈现,不裁剪)。"""
    ttf_cands = [
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    ]
    body = None
    for path in ttf_cands:
        if not os.path.isfile(path):
            continue
        is_ttc = path.lower().endswith(".ttc")
        indices = range(4) if is_ttc else range(1)
        for idx in indices:
            try:
                name = "CJKBody_%s_%d" % (os.path.basename(path).
                                          replace(".", "_"), idx)
                if is_ttc:
                    f = TTFont(name, path, subfontIndex=idx)
                else:
                    f = TTFont(name, path)
                pdfmetrics.registerFont(f)
                body = name
                break
            except Exception:                           # noqa: BLE001
                continue
        if body:
            break
    if body is None:
        raise RuntimeError(
            "未找到可嵌入的 TrueType 轮廓 CJK 字体(系统 Noto CJK 为 CFF "
            "轮廓,reportlab 不支持)")
    mono = body          # 无 CJK 等宽 TTF,如实退回正文字体
    return body, mono


def wrap_line(line, width=108):
    """超宽代码行折行(不裁剪内容)。"""
    out = []
    while len(line) > width:
        out.append(line[:width])
        line = "    " + line[width:]
    out.append(line)
    return out


def iter_source_files(ws_src):
    """全部包文本文件(排序稳定);二进制/超大文件单列。"""
    texts, bins = [], []
    for root, _dirs, files in os.walk(ws_src):
        for fn in sorted(files):
            p = os.path.join(root, fn)
            rel = os.path.relpath(p, ws_src)
            if os.path.islink(p):
                continue
            if os.path.getsize(p) > 2 * 1024 * 1024:
                bins.append((rel, "超过 2MB,跳过内容,仅记校验和"))
                continue
            ext = os.path.splitext(fn)[1].lower()
            if ext not in TEXT_EXT:
                bins.append((rel, "二进制/非文本扩展名,跳过内容"))
                continue
            try:
                with open(p, "r", encoding="utf-8") as f:
                    data = f.read()
                texts.append((rel, data))
            except (UnicodeDecodeError, ValueError):
                bins.append((rel, "非 UTF-8 文本,跳过内容"))
    texts.sort(key=lambda x: x[0])
    bins.sort()
    return texts, bins


class ReportDoc(BaseDocTemplate):
    def afterFlowable(self, flowable):
        if isinstance(flowable, Paragraph):
            st = getattr(flowable, "style", None)
            if st and st.name in ("H1", "H2"):
                text = flowable.getPlainText()
                self.notify("TOCEntry", (
                    0 if st.name == "H1" else 1, text, self.page))


def build(ws, out_dir, data):
    body_font, mono_font = register_cjk()
    today = _dt.date.today().isoformat()
    os.makedirs(out_dir, exist_ok=True)
    pdf_path = os.path.join(
        out_dir, "RoboSub_Controller_System_Full_Source_Report_%s.pdf"
        % today)

    ss = {
        "body": ParagraphStyle("body", fontName=body_font, fontSize=9,
                               leading=13.5, spaceAfter=4),
        "H1": ParagraphStyle("H1", fontName=body_font, fontSize=14,
                             leading=18, spaceBefore=10, spaceAfter=6,
                             textColor=colors.HexColor("#1a3a5c")),
        "H2": ParagraphStyle("H2", fontName=body_font, fontSize=11,
                             leading=15, spaceBefore=8, spaceAfter=4),
        "code": ParagraphStyle("code", fontName=mono_font, fontSize=5.6,
                               leading=7.4),
        "tocH": ParagraphStyle("tocH", fontName=body_font, fontSize=12,
                               leading=16, spaceBefore=6),
    }
    doc = ReportDoc(pdf_path, pagesize=A4, leftMargin=16 * mm,
                    rightMargin=14 * mm, topMargin=16 * mm,
                    bottomMargin=15 * mm, title="RoboSub 控制器系统全源码报告"
                    " (%s)" % today)

    def footer(canv, _doc):
        canv.saveState()
        canv.setFont(body_font, 7.5)
        canv.setFillColor(colors.HexColor("#666666"))
        canv.drawString(16 * mm, 8 * mm,
                        "RoboSub Controller System Full Source Report %s"
                        % today)
        canv.drawRightString(A4[0] - 14 * mm, 8 * mm,
                             "第 %d 页" % canv.getPageNumber())
        canv.restoreState()

    fr = Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height,
               id="f")
    doc.addPageTemplates([PageTemplate(id="pt", frames=[fr],
                                       onPage=footer)])
    story = []
    P = lambda t, s="body": Paragraph(t, ss[s])        # noqa: E731

    # ---- 目录
    story.append(P("目录", "H1"))
    toc = TableOfContents()
    toc.levelStyles = [ss["tocH"], ParagraphStyle(
        "toc2", fontName=body_font, fontSize=9.5, leading=13,
        leftIndent=14)]
    story.append(toc)
    story.append(Spacer(1, 8))

    # ---- 正文 13 章(内容如实描述实现;验证数字取 data)
    story.append(P("1. 执行环境与系统概览", "H1"))
    story.append(P(
        "本报告覆盖服务器 %s 上的工作区 <code>%s</code>。执行环境:"
        "Ubuntu 24.04,ROS 2 Jazzy,Isaac Sim 5.1.0(/isaacsim,"
        "python.sh=Python 3.11 + torch 2.7.0+cu128),NVIDIA RTX 5080。"
        "工作区无 git 仓库;实施前已做源码快照归档(见 §11 与附录 B)。"
        % (data.get("server", ""), ws), "body"))
    env_rows = [["项目", "值"],
                ["ROS 2", data.get("ros", "Jazzy")],
                ["Isaac Sim", data.get("isaac", "5.1.0-rc.19 (/isaacsim)")],
                ["Isaac python", data.get("isaac_py",
                                          "3.11.13 + torch 2.7.0+cu128")],
                ["系统 python", data.get("sys_py", "3.12 (ROS 节点)")],
                ["GPU", data.get("gpu", "RTX 5080 16GB")],
                ["物理阶段", "Zero-Gravity Damped(重力 0,浮力关,"
                             "体轴阻尼,40 kg)"]]
    t = Table(env_rows, colWidths=[38 * mm, 130 * mm])
    t.setStyle(TableStyle([
        ("FONTNAME", (0, 0), (-1, -1), body_font),
        ("FONTSIZE", (0, 0), (-1, -1), 8.5),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#dde7f0")),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#99a"))]))
    story.append(t)

    story.append(P("2. 工作区与包结构", "H1"))
    story.append(P("既有包(保留并按需扩展):auv_controller_MPPI(C++,"
                   "未改动)、auv_isaac_bridge(扩展:扰动力/可视化/多写者"
                   "防护/进程内 API)、auv_mission_control(未改动)、"
                   "auv_sensing(未改动)。新增包:auv_common(推进器几何"
                   "单一真源 + 坐标工具 + 有界分配)、auv_controller_pid、"
                   "auv_controller_gru(推理 + 训练)、auv_experiments("
                   "实验/指标/绘图/整定/本报告)、auv_bringup(统一启动)。"
                   "所有新配置集中于各包 config/,推进器几何不重复维护"
                   "(auv_common/thrusters.yaml 为单一真源,与 MPPI 原"
                   "yaml 的一致性由单元测试锁定)。", "body"))

    story.append(P("3. 坐标系与接口约定", "H1"))
    story.append(P("体坐标系:+x 左舷,+y 上,+z 艇体纵轴(前);roll 绕 z、"
                   "pitch 绕 x、yaw 绕 y;四元数 (x,y,z,w),体->世界旋转。"
                   "话题:/auv/state/control 与 /auv/ground_truth/odometry"
                   "(nav_msgs/Odometry,位置为质心系,速度 twist 为体系),"
                   "/auv/control/goal(geometry_msgs/PoseStamped,世界系),"
                   "/auv/control/thruster_cmd(std_msgs/Float64MultiArray,"
                   "8 维,牛顿,序号与 thrusters.yaml t0..t7 一致),/clock。"
                   "QoS:状态=SensorDataQoS,命令=Reliable depth10,"
                   "状态 60Hz/命令 50Hz。", "body"))

    story.append(P("4. 推进器配置与有界分配", "H1"))
    story.append(P("8 推进器:0-3 水平 45° 对角,4-7 垂直 +y;安装位置相对"
                   "质心估计(USD 实测)。B=[d; r×d] 为 6×8,秩 6(可观"
                   "控三轴力+三轴力矩)。单机限幅 ±39.2266 N。反分配用"
                   "迭代饱和-重解的加权最小二乘(auv_common.alloc."
                   "allocate_bounded),并带 scipy lsq_linear 交叉验证单测;"
                   "饱和时记录 requested/achieved wrench 差与饱和占比。"
                   "<b>质心是碰撞网格体积质心估计,非实测,全链路未二次"
                   "偏移</b>;桥与控制器共用同一原点约定(见 §1 快照内"
                   " isaac_bridge.json 注释)。", "body"))

    story.append(P("5. PID 控制器(6 自由度)", "H1"))
    story.append(P("世界系位置误差转体系(R^T(p_d−p)),姿态误差用相对"
                   "旋转旋转向量(log(R^T R_d),非欧拉角相减),微分含"
                   "旋转项 R^T(v_d−v)−ω×e;条件积分+积分限幅+输出限幅同向"
                   "停积+分配饱和回衰;微分一阶低通;YAML 参数化;无在线"
                   "质心自适应;前馈默认关闭(开启属增强版并记录)。整定"
                   "流程(auv_experiments/tune_pid.py):零输入保位 -> "
                   "P(平移/转动分组)-> D -> I(仅稳态误差存在时)-> 组合"
                   "-> 固定扰动 -> 独立验证集;发散/OOB/预算中止。最佳参数"
                   "文件:%s" % data.get("pid_yaml", "(待整定产出)"),
                   "body"))

    story.append(P("6. GRU 神经控制器", "H1"))
    story.append(P("观测 23 维(0-2 体位置误差,3-5 姿态误差旋转向量,"
                   "6-8 体系线速度,9-11 体系角速度,12-14 垂直参考体系"
                   "方向,15-22 上一周期实际执行限幅后归一化推力),顺序由"
                   "obs_builder.OBS_LAYOUT 与单测锁定;网络 encoder "
                   "Linear(23,64)+SiLU -> GRU(64,64) -> concat[64,23]=87 "
                   "-> 动作头 87-64-64 -> Beta(α,β)×8,动作 a=2x−1∈[−1,1]"
                   "映射 f=a·Fmax(物理推力,非 PWM);训练为自定义循环 PPO"
                   "(Beta 分布,与有界动作密度匹配;GAE、chunk 序列更新、"
                   "隐状态跨周期、回合边界复位;训练/评测种子分离;检查点/"
                   "导出 npz+元数据;单环境同步)。隐藏状态仅在回合复位/"
                   "重接管/时间回跳/位置跳变时清零,目标更新不清零。产物:"
                   "%s;训练状态:%s" % (
                       data.get("gru_model", "(待训练)"),
                       data.get("gru_train", "(未开始)")), "body"))

    story.append(P("7. MPPI 保留与统一启动互斥", "H1"))
    story.append(P("auv_controller_MPPI 未改动。统一入口 auv_bringup/"
                   "bringup.launch.py 以 controller:=pid|mppi|gru 选择,"
                   "同一时刻仅一个控制器节点被启动;goal 源(hold|keyboard|"
                   "experiment|none)互斥参数化。运行期冗余防线:pid/gru "
                   "节点发 /auv/control/heartbeat 心跳,互相检测即停发;"
                   "桥侧 1Hz 检查 thruster_cmd 发布者数,&gt;1 且"
                   " enforce_single_controller 时输出置零并在 status 标注"
                   " fault(MPPI C++ 不可改,防线在桥侧)。", "body"))

    story.append(P("8. 双作用点随机扰动力", "H1"))
    story.append(P("作用点为既有纯 Xform prim /World/Barracuda_2/"
                   "barracuda2/base_link/disturbance_{0,1}(无刚体/碰撞/"
                   "质量 API),力经 apply_force_at_pos 施加于所属刚体,"
                   "力矩由 PhysX 自然生成(r×F 不重复施加)。每股独立:"
                   "方向球面均匀(正态采样归一化),幅值均匀[0,19.6133 N]"
                   "(上限约束向量模长),余弦平滑过渡(范数凸性保证插值"
                   "不越界),重采样间隔/过渡时长可配,默认体系(世界系可"
                   "选),seed 可复现,支持序列回放与 step/pulse 测试模"
                   "式;三控制器共用同一实现(桥内 DisturbanceManager)。"
                   "实测历史经 /auv/disturbance/forces(12=[F0,F1,P0,P1])"
                   "发布并落盘。", "body"))

    story.append(P("9. Isaac 视窗实时可视化", "H1"))
    story.append(P("isaacsim.util.debug_draw 线段绘制于 Isaac 视窗"
                   "(非 RViz):8 推进器力=蓝,2 扰动力=红,统一比例默认"
                   "1 m↔40 N(可配),负推力方向反转,零力隐藏,超长裁剪"
                   "并在 status/viz_get 如实报 clipped;当前位姿=黄、目标"
                   "位姿=绿(定长箭头沿体 +z 前,附三轴姿态小标,不代表"
                   "推力)。箭头自实际作用点、实际方向绘制;清空重画更新,"
                   "不逐帧新增 prim;暂停/复位/关闭清理;速率可配;headless"
                   "训练配置默认关闭。", "body"))

    story.append(P("10. 实验系统与指标", "H1"))
    story.append(P("auv_experiments:任务库(保位/单轴双向/偏航俯仰横滚/"
                   "组合/扰动变体,三控制器同一任务集)+ 编排(桥 TCP 复位"
                   "序列:pause->set_pose->zero->扰动配置->控制器复位->"
                   "play->发目标->判据)-> 记录(逐回合 states/goals/"
                   "thrust/disturbance CSV + diag.jsonl + 可选 rosbag2 + "
                   "metadata.json)-> 指标(RMSE/最大误差/首达容差/稳定"
                   "时间/稳态误差/超调/速度 RMS/推力变化/饱和占比/扰动峰"
                   "值偏差与恢复时间/成功率/推理 P95P99;推力平方积分仅作"
                   "控制代价代理)-> 绘图 PNG(+PDF/SVG 矢量)。时间约定:"
                   "运动量=仿真时间,推理耗时=单调墙钟;仿真时间跳变切分"
                   "回合;在线指标与离线 analyze.py 复算同源并可对账。"
                   "产物路径:%s" % data.get("runs", "(见服务器 runs/)"),
                   "body"))

    story.append(P("11. 验证结果汇总", "H1"))
    for line in data.get("verification", [
            "(尚未注入验证结果 — 运行后由 report_data.json 提供)"]):
        story.append(P("• " + line))
    story.append(P("实施前快照:%s(附录 B 含全部文件 SHA-256)。"
                   % data.get("snapshot", "auv_ws_src_snapshot_20261107"
                                          ".tar.gz"), "body"))

    story.append(P("12. 已知限制与诚实声明", "H1"))
    for line in data.get("limits", [
        "质心为体积质心估计(非实测),报告与代码注释均按估计表述;",
        "推进器方向正负号来自 USD 局部 +X 推导,最终以逐推进器小推力"
        "实测为准(配置热重载可改);",
        "GRU 训练为短程预算验证(管线可用),不宣称收敛或达标;",
        "扰动默认参数为初版经验值,未宣称最优;",
        "系统 python matplotlib 与 pyzed numpy 冲突,绘图/PDF 统一走 "
        "Isaac python。"]):
        story.append(P("• " + line))

    story.append(P("13. 附录 A:完整源代码", "H1"))
    story.append(P("范围:工作区 src/ 全部包(既有 + 新增)全部文本文件,"
                   "逐文件路径标题;秘密模式打码;超长行折行不截断。"
                   "二进制文件跳过内容、保留清单与校验和(附录 B)。",
                   "body"))
    texts, bins = iter_source_files(os.path.join(ws, "src"))
    story.append(P("文本文件 %d 个,二进制/跳过 %d 个。附录 B 含全部"
                   "校验和。" % (len(texts), len(bins))))

    # 附录 A(正文与附录同一文档;TOC 需要 multiBuild 多遍填充)
    story.append(Spacer(1, 6))
    for rel, content in texts:
        story.append(P("文件: src/%s" % rel, "H2"))
        lines = []
        for ln in mask_secrets(content).splitlines():
            lines.extend(wrap_line(ln.replace("\t", "    ")))
        # Preformatted(非 XPreformatted):不解析 XML 标记,源码里的
        # <br>/<script> 等按字面呈现(XPreformatted 会当标记解析而抛
        # paraparser 语法错误,2026-11-08 实测于 web_keyboard.py)。
        story.append(Preformatted("\n".join(lines), ss["code"]))
        story.append(Spacer(1, 4))

    story.append(P("附录 B:文件清单与 SHA-256 校验和", "H1"))
    rows = [["文件", "SHA-256(前 16)"]]
    manifest = {}
    for rel, content in texts:
        h = hashlib.sha256(content.encode("utf-8")).hexdigest()
        manifest["src/" + rel] = h
        rows.append(["src/" + rel, h[:16]])
    for rel, note in bins:
        p = os.path.join(ws, "src", rel)
        h = hashlib.sha256(open(p, "rb").read()).hexdigest()
        manifest["src/" + rel] = h
        rows.append(["src/" + rel + " (%s)" % note, h[:16]])
    t = Table(rows, colWidths=[120 * mm, 48 * mm], repeatRows=1)
    t.setStyle(TableStyle([
        ("FONTNAME", (0, 0), (-1, -1), mono_font),
        ("FONTSIZE", (0, 0), (-1, -1), 5.5),
        ("LEADING", (0, 0), (-1, -1), 7),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#dde7f0")),
        ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#aab"))]))
    story.append(t)
    with open(os.path.join(out_dir, "manifest.json"), "w",
              encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)

    doc.multiBuild(story)

    # ---- 结构化渲染校验(诚实:字节级结构检查,非视觉检查)
    with open(pdf_path, "rb") as f:
        raw = f.read()
    n_pages = raw.count(b"/Type /Page") - raw.count(b"/Type /Pages")
    check = {
        "pdf": pdf_path,
        "size_bytes": len(raw),
        "header_ok": raw[:5] == b"%PDF-",
        "eof_ok": b"%%EOF" in raw[-2048:],
        "pages_detected": int(n_pages),
        "ok": raw[:5] == b"%PDF-" and b"%%EOF" in raw[-2048:] and
              n_pages > 10,
    }
    with open(os.path.join(out_dir, "render_check.json"), "w",
              encoding="utf-8") as f:
        json.dump(check, f, ensure_ascii=False, indent=2)
    print(json.dumps(check, ensure_ascii=False))
    return pdf_path, check


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--data", default="")
    args = ap.parse_args()
    data = {}
    if args.data and os.path.isfile(args.data):
        data = json.load(open(args.data, encoding="utf-8"))
    build(os.path.abspath(args.ws), os.path.abspath(args.out), data)


if __name__ == "__main__":
    main()
