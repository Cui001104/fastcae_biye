# 《基于FastCAE的齿轮CAD/CAE自动分析与优化平台——第二阶段研究进展》

风格：蓝白学术风，白底、深蓝标题、浅蓝流程线。每页少文字，代码模块名作为页脚小字注释。

---

# 1. 封面

## 标题

基于 FastCAE 的齿轮 CAD/CAE 自动分析与优化平台  
第二阶段研究进展

## 页面要点

- FastCAE 平台基础上的新增功能
- RBF 代理模型
- CalculiX 工况优化
- Gmsh 网格加密
- 有限元结果验证与论文进展

## 建议图示

```text
FastCAE平台底座
        ↓
GearAutoOpt新增模块
        ↓
RBF + CCX + Gmsh + SQLite + NSGA-II
```

## 代码注释

`src/GearAutoOpt/` · `src/GearAutoOpt/CMakeLists.txt`

---

# 2. 第二阶段总体进展：自动闭环 → 智能闭环

## 页面要点

- 第一阶段：自动建模、网格、求解、NSGA-II、SQLite
- 第二阶段：代理预测 + 稀疏加点 + CCX 验证
- 求解端：接触、刚体、位移驱动、失败分类
- 网格端：齿根加密、集合丰富化、接触面校验

## 建议图示

```text
第一次汇报
参数 → 建模 → 网格 → CCX → 结果 → NSGA-II

第二阶段新增
SQLite样本 → RBF训练 → 代理预测 → 稀疏加点 → CCX验证 → 误差反馈
```

## 代码注释

`GearAutoOptManager::startSurrogateAssisted` · `GearSurrogateModel::train` · `MeshConverter::enrichGearMeshInp`

---

# 3. RBF代理模型模块

## 页面要点

- 独立模型类：`GearSurrogateModel`
- 输入：8 个修形与结构变量
- 输出：应力、位移、质量预测
- 核函数：高斯 RBF

## 建议图示

```text
GearDesignPoint
   ↓ surrogateInputVars
[x1, x2, ca1, lca1, ca2, lca2, width, hubRatio]
   ↓
RBF Model
   ↓
sigmaMaxPred / uMaxPred / massPred
```

## 代码注释

`GearSurrogateModel::train` · `predict` · `solveRbfWeights` · `kernel`

---

# 4. 代理辅助NSGA-II优化流程

## 页面要点

- 保留原 NSGA-II 主流程
- 新增代理辅助入口
- 样本不足：LHS + CCX 补样
- 代理 Pareto 后进行稀疏加点
- CCX 验证后计算 RMAE

## 建议图示

```text
验证样本
  ↓
RBF训练
  ↓
代理NSGA-II演化
  ↓
代理Pareto
  ↓
稀疏加点
  ↓
CCX真实验证
  ↓
RMAE误差判断
```

## 代码注释

`startSurrogateAssisted` · `evaluatePopulationBySurrogate` · `GearInfillSelector::selectSparseParetoPoints` · `computeRmae`

---

# 5. SQLite样本库与结果复用

## 页面要点

- 结果库：`GearOptResults.db`
- 全局库 + 本次运行库
- 只加载 CCX 验证样本训练代理模型
- 记录代理使用、验证状态、Pareto 标记

## 建议图示

```text
CCX计算结果
   ↓ insertDesignPointResult
GearOptResults.db
   ├─ global()
   └─ runSession()
   ↓ loadValidatedSamples
RBF训练样本
```

## 代码注释

`GearOptResultDatabase::global` · `runSession` · `insertDesignPointResult` · `loadValidatedSamples`

---

# 6. CalculiX工况写入与求解稳定性

## 页面要点

- 自动生成 `job.inp`
- 支持 `*RIGID BODY`
- 支持 `*CONTACT PAIR`
- 支持 `*SURFACE INTERACTION`
- 识别 CCX 失败原因

## 建议图示

```text
mesh.inp + 工况参数
        ↓
InpContext
        ↓
writeJobInp → job.inp
        ↓
CCXSolverController
        ↓
ccx_MT.exe
        ↓
成功 / 超时 / 不收敛 / 奇异矩阵 / 负雅可比
```

## 代码注释

`CCXInpWriter::writeJobInp` · `renderJobInp` · `CCXSolverController::start` · `failureReasonToString`

---

# 7. Gmsh网格生成与局部加密

## 页面要点

- 自动生成 `gear.geo`
- 调用 Gmsh 输出 `mesh.inp`
- 齿根曲线识别
- `Distance + Threshold` 局部加密
- 生成齿面与轮毂集合

## 建议图示

```text
gear1.brep / gear2.brep
        ↓
gear.geo
        ↓ Gmsh -3 -format inp
mesh.inp
        ↓ stripSurfaceElements
enriched mesh.inp
        ↓
HUB / TOOTH_OUTER / TOOTH_SURF / master-slave
```

## 代码注释

`GearOptCaseRunner::runMeshStep` · `writeGearGeoContent` · `collectRootCurveTagsForGmsh` · `MeshConverter::enrichGearMeshInp`

---

# 8. 有限元结果解析与误差评价

## 页面要点

- 优先解析 `job.frd`
- 回退解析 `job.dat`
- 提取最大 von Mises 与最大位移
- 导出 `result.vtu`
- RMAE 与接触面校验

## 建议图示

```text
CCX输出
 ├─ job.frd → U / S → max von Mises / max displacement
 ├─ job.dat → fallback stress
 ├─ result.vtu → ParaView
 └─ RMAE + SurfaceVerify → 误差评价支撑
```

## 代码注释

`CCXResultParser::parseFrd` · `parseDat` · `maxVonMisesForNodes` · `exportCalculixResultToVTK` · `verifyCcxElementSurfaceFaces`

---

# 9. 代码完成度评估

## 页面要点

- 已完成：RBF、代理加点、SQLite 样本库
- 已完成：CCX 工况、Gmsh 加密、结果解析
- 工程化：失败分类、VTU 导出、GUI 参数入口
- 待补充：理论公式验证、网格收敛图表、代理误差曲线

## 建议图示

```text
完成度矩阵

模块        实现    可写入论文    后续补充
RBF         已完成  高           验证曲线
CCX工况     已完成  高           参数对比
Gmsh网格    已完成  高           网格收敛
结果验证    部分完成 中           Hertz/AGMA对比
```

## 代码注释

`GearSurrogateModel` · `CCXInpWriter` · `MeshConverter` · `CCXResultParser` · `CalculiXResultVtkExport`

---

# 10. 论文编写进展：代码到章节映射

## 页面要点

- 系统设计：设计点、配置、总控流程
- 代理优化：RBF、稀疏加点、NSGA-II
- 有限元求解：网格、INP、CCX 控制
- 结果验证：FRD/DAT、VTU、RMAE

## 建议图示

```text
代码模块                         论文章节
GearDesignPoint / Config   →   系统设计
GearSurrogateModel / NSGA2 →   代理优化方法
GearOptCaseRunner / CCX    →   自动有限元求解
CCXResultParser / VTU      →   结果分析与验证
```

## 代码注释

`GearDesignPoint` · `GearOptConfig` · `GearAutoOptManager` · `GearSurrogateModel` · `CCXResultParser`

---

# 11. 当前问题与下一步计划

## 页面要点

- 增加 CCX 验证样本
- 绘制 RMAE 误差曲线
- 整理网格收敛数据
- 补充理论公式对比
- 完善论文实验章节

## 建议图示

```text
样本扩充
   ↓
代理误差曲线
   ↓
网格收敛分析
   ↓
理论公式对比
   ↓
论文定稿
```

## 代码注释

`startSurrogateAssisted` · `computeRmae` · `GearDesignPoint::meshSize_mm` · `CCXFailureReason`

---

# 12. 阶段性总结

## 页面要点

- 形成代理辅助优化闭环
- 完善双齿轮 CalculiX 接触工况
- 完成 Gmsh 齿根加密与集合生成
- 完成 FRD/DAT 解析与 VTU 导出
- 论文进入实验图表与验证补充阶段

## 建议图示

```text
第二阶段成果

RBF代理模型
   + NSGA-II
   + SQLite样本库
   + CCX求解工况
   + Gmsh网格优化
   + 结果解析验证
        ↓
毕业设计最终系统与论文基础
```

## 代码注释

`src/GearAutoOpt/surrogate/` · `src/GearAutoOpt/runner/` · `src/GearAutoOpt/solver/` · `src/GearAutoOpt/db/`

