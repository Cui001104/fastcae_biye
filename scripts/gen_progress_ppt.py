# -*- coding: utf-8 -*-
"""生成「总体进展」单页 PPT，风格对齐参考图。"""
from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_AUTO_SHAPE_TYPE, MSO_CONNECTOR
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt

OUT = Path(__file__).resolve().parents[1] / "docs" / "总体进展_代理辅助优化.pptx"

# 配色（参考图：深蓝标题 + 浅蓝侧栏 + 白底流程）
C_TITLE_BG = RGBColor(0x1A, 0x3A, 0x5C)
C_WHITE = RGBColor(0xFF, 0xFF, 0xFF)
C_TEXT = RGBColor(0x1E, 0x1E, 0x1E)
C_SUB = RGBColor(0x55, 0x55, 0x55)
C_PHASE1 = RGBColor(0xE8, 0xF4, 0xFC)
C_PHASE2 = RGBColor(0xE3, 0xF2, 0xFD)
C_SIDEBAR = RGBColor(0xD6, 0xEA, 0xF8)
C_BOX = RGBColor(0xFF, 0xFF, 0xFF)
C_BORDER = RGBColor(0x90, 0xCA, 0xF9)
C_ARROW = RGBColor(0x64, 0xB5, 0xF6)
C_ACCENT = RGBColor(0x19, 0x76, 0xD2)
C_CSV = RGBColor(0xE8, 0xF5, 0xE9)
C_FEEDBACK = RGBColor(0xC8, 0xE6, 0xC9)


def set_run(p, text, size=11, bold=False, color=C_TEXT):
    p.text = text
    p.font.size = Pt(size)
    p.font.bold = bold
    p.font.name = "Microsoft YaHei"
    p.font.color.rgb = color
    p.alignment = PP_ALIGN.CENTER


def add_box(slide, x, y, w, h, title, sub="", fill=C_BOX, line=C_BORDER, title_size=10):
    sh = slide.shapes.add_shape(MSO_AUTO_SHAPE_TYPE.ROUNDED_RECTANGLE, x, y, w, h)
    sh.fill.solid()
    sh.fill.fore_color.rgb = fill
    sh.line.color.rgb = line
    sh.line.width = Pt(1)
    tf = sh.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    set_run(p, title, title_size, True)
    if sub:
        p2 = tf.add_paragraph()
        set_run(p2, sub, 8, False, C_SUB)
    return sh


def add_arrow(slide, x1, y1, x2, y2, dashed=False):
    conn = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, x1, y1, x2, y2)
    conn.line.color.rgb = C_ARROW
    conn.line.width = Pt(1.5)
    if dashed:
        conn.line.dash_style = 3
    return conn


def add_phase_band(slide, y, h, label, sub, fill):
    band = slide.shapes.add_shape(MSO_AUTO_SHAPE_TYPE.RECTANGLE, Inches(0.15), y, Inches(9.55), h)
    band.fill.solid()
    band.fill.fore_color.rgb = fill
    band.line.fill.background()
    tf = band.text_frame
    tf.margin_left = Inches(0.08)
    p = tf.paragraphs[0]
    set_run(p, label, 12, True, C_ACCENT)
    p.alignment = PP_ALIGN.LEFT
    p2 = tf.add_paragraph()
    set_run(p2, sub, 9, False, C_SUB)
    p2.alignment = PP_ALIGN.LEFT


def main():
    OUT.parent.mkdir(parents=True, exist_ok=True)
    prs = Presentation()
    prs.slide_width = Inches(10)
    prs.slide_height = Inches(7.5)
    slide = prs.slides.add_slide(prs.slide_layouts[6])

    # 标题栏
    header = slide.shapes.add_shape(
        MSO_AUTO_SHAPE_TYPE.RECTANGLE, Inches(0), Inches(0), Inches(10), Inches(0.72)
    )
    header.fill.solid()
    header.fill.fore_color.rgb = C_TITLE_BG
    header.line.fill.background()
    hp = header.text_frame.paragraphs[0]
    set_run(hp, "总体进展：从自动化分析到代理模型辅助智能优化", 20, True, C_WHITE)

    # 右侧摘要栏
    sb = slide.shapes.add_shape(
        MSO_AUTO_SHAPE_TYPE.ROUNDED_RECTANGLE, Inches(7.05), Inches(0.85), Inches(2.75), Inches(6.45)
    )
    sb.fill.solid()
    sb.fill.fore_color.rgb = C_SIDEBAR
    sb.line.color.rgb = C_BORDER
    tf = sb.text_frame
    tf.word_wrap = True
    tf.margin_left = Inches(0.12)
    tf.margin_right = Inches(0.08)
    items = [
        ("第一阶段", "参数设计 → 自动建模 → Gmsh 网格\n→ CCX 求解 → 结果解析\n→ NSGA-II → SQLite 样本库"),
        ("第二阶段（代码）", "baseCaseHash 筛样本 → RBF 训练\n→ 代理 NSGA-II → 稀疏加点\n→ CCX 验证 → RMAE/HV 导出 CSV\n→ 未达标则重训 RBF"),
        ("SQLite 样本库", "global() + runSession()\nbaseCaseHash / designHash\nverified_by_ccx = 1"),
        ("运行目录 CSV", "surrogate_metrics.csv\n  PredHV / PredPareto\nccx_metrics.csv\n  CCXSamples / CCXHV\nsurrogate_rmae.csv\n  RMAE_σ / RMAE_u"),
        ("求解端", "接触 / 刚体 / 位移驱动\nStep-2 扭矩映射 CLOAD"),
        ("网格端", "齿根加密 / 集合丰富化\n接触面校验"),
    ]
    for i, (t, b) in enumerate(items):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        if i > 0:
            p.space_before = Pt(8)
        set_run(p, t, 10, True, C_ACCENT)
        p.alignment = PP_ALIGN.LEFT
        p2 = tf.add_paragraph()
        set_run(p2, b, 8, False, C_TEXT)
        p2.alignment = PP_ALIGN.LEFT

    # ===== 第一阶段 =====
    add_phase_band(
        slide,
        Inches(0.82),
        Inches(0.38),
        "第一阶段 — 自动闭环",
        "已完成：自动化建模到优化闭环（GearAutoOptManager::start）",
        C_PHASE1,
    )
    y1 = Inches(1.28)
    bw, bh, gap = Inches(0.82), Inches(0.72), Inches(0.12)
    x0 = Inches(0.22)
    p1 = [
        ("参数设计", "UI 工况"),
        ("自动建模", "OCC 齿轮"),
        ("网格生成", "Gmsh"),
        ("CCX 求解", "CalculiX"),
        ("结果解析", "FRD/DAT"),
        ("NSGA-II", "全 CCX 评估"),
        ("SQLite", "样本库"),
    ]
    boxes1 = []
    for i, (t, s) in enumerate(p1):
        x = x0 + i * (bw + gap)
        boxes1.append(add_box(slide, x, y1, bw, bh, t, s))
        if i > 0:
            add_arrow(slide, x - gap, y1 + bh / 2, x, y1 + bh / 2)
    # 闭环虚线
    add_arrow(
        slide,
        x0 + 6 * (bw + gap) + bw / 2,
        y1 + bh,
        x0 + bw / 2,
        y1 + bh,
        dashed=True,
    )

    # ===== 第二阶段 =====
    add_phase_band(
        slide,
        Inches(2.18),
        Inches(0.42),
        "第二阶段 — 智能闭环",
        "新增：startSurrogateAssisted() — 同 baseCaseHash 样本 + 代理预测 + CCX 验证 + CSV 指标",
        C_PHASE2,
    )
    y2 = Inches(2.72)
    bw2, bh2, gap2 = Inches(0.72), Inches(0.78), Inches(0.08)
    x2 = Inches(0.18)
    p2 = [
        ("UI 勾选", "代理辅助"),
        ("SQLite", "baseCase\n筛选"),
        ("样本检查", "≥25?\nLHS补样"),
        ("RBF 训练", "8D 输入"),
        ("代理\nNSGA-II", "RBF 评估"),
        ("surrogate_\nmetrics", "PredHV\nPredPareto"),
        ("稀疏加点", "Infill"),
        ("CCX 验证", "infill 点"),
        ("surrogate_\nrmae", "RMAE_σ/u"),
        ("ccx_\nmetrics", "CCXHV\n样本数"),
    ]
    boxes2 = []
    for i, (t, s) in enumerate(p2):
        x = x2 + i * (bw2 + gap2)
        fill = C_CSV if "metrics" in t or "rmae" in t else C_BOX
        boxes2.append(add_box(slide, x, y2, bw2, bh2, t, s, fill=fill, title_size=8))
        if i > 0:
            add_arrow(slide, x - gap2, y2 + bh2 / 2, x, y2 + bh2 / 2)

    # 反馈与循环标注
    note = slide.shapes.add_shape(
        MSO_AUTO_SHAPE_TYPE.ROUNDED_RECTANGLE, Inches(0.22), Inches(3.62), Inches(6.65), Inches(0.55)
    )
    note.fill.solid()
    note.fill.fore_color.rgb = C_FEEDBACK
    note.line.color.rgb = RGBColor(0x66, 0xBB, 0x6A)
    np = note.text_frame.paragraphs[0]
    set_run(
        np,
        "循环：RMAE_σ & RMAE_u < 5% 或达 maxGenerations → 停止；否则 CCX 新样本写入 SQLite → 重训 RBF → 下一轮。"
        " 最终 Pareto 仅取 status=Done 的 CCX 真值。",
        9,
        False,
        C_TEXT,
    )

    # 反馈箭头：ccx_metrics → SQLite；RMAE → RBF
    add_arrow(slide, x2 + 7 * (bw2 + gap2) + bw2 / 2, y2, x2 + 1 * (bw2 + gap2) + bw2 / 2, y2 - Inches(0.05), dashed=True)
    add_arrow(slide, x2 + 8 * (bw2 + gap2) + bw2 / 2, y2 + bh2, x2 + 3 * (bw2 + gap2) + bw2 / 2, y2 + bh2 + Inches(0.08), dashed=True)

    fb = slide.shapes.add_shape(MSO_AUTO_SHAPE_TYPE.OVAL, Inches(5.95), Inches(4.35), Inches(1.05), Inches(0.55))
    fb.fill.solid()
    fb.fill.fore_color.rgb = RGBColor(0xA5, 0xD6, 0xA7)
    fb.line.color.rgb = RGBColor(0x43, 0xA0, 0x47)
    fp = fb.text_frame.paragraphs[0]
    set_run(fp, "误差反馈\nRMAE", 8, True, C_TEXT)

    # 底部图例
    leg = slide.shapes.add_textbox(Inches(0.22), Inches(5.05), Inches(6.6), Inches(0.35))
    lp = leg.text_frame.paragraphs[0]
    set_run(
        lp,
        "■ 绿色框 = 运行目录 CSV（不入 SQLite）    ■ 虚线 = 样本/模型更新回路    ■ 实线 = 单轮主流程",
        8,
        False,
        C_SUB,
    )
    lp.alignment = PP_ALIGN.LEFT

    prs.save(str(OUT))
    print(f"Saved: {OUT}")


if __name__ == "__main__":
    main()
