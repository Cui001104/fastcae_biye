# 《基于FastCAE的齿轮CAD/CAE自动分析与优化平台——第二阶段研究进展》PPT大纲

整体风格建议：蓝白学术风格，白底为主，深蓝标题栏，浅蓝流程线，代码模块名作为页脚小字注释。每页控制 3-5 个短要点，图示优先于大段文字。

---

# 1. 封面：第二阶段研究进展

## 本页核心内容

- 主题：基于 FastCAE 的齿轮 CAD/CAE 自动分析与优化平台
- 第二阶段重点：代理模型、求解工况、网格加密、结果验证、论文整理
- FastCAE 仅作为平台底座，不再重复介绍基础框架

### 对应代码模块

- `src/GearAutoOpt/`
- `src/GearAutoOpt/CMakeLists.txt`
- `docs/结构说明.md`

### 建议流程图

- 简化为一行：`FastCAE平台底座 -> GearAutoOpt新增模块 -> 第二阶段研究成果`

### 建议图片

- 蓝白背景 + 齿轮轮廓线稿 + 模块关键词：RBF / CCX / Gmsh / SQLite / NSGA-II

### 建议演讲内容

本次汇报不再重复第一阶段已经完成的平台接入、参数化建模和基础自动流程，而是聚焦在 FastCAE 平台之上的新增研究工作。第二阶段主要围绕五个方面展开：一是引入 RBF 代理模型以降低有限元调用成本；二是优化 CalculiX 求解工况，使双齿轮接触和刚体轮毂约束更稳定；三是补充 Gmsh 网格局部加密与网格集合后处理；四是完善有限元结果解析、可视化和误差评价；五是把代码结构和论文内容进行对应整理。

---

# 2. 第二阶段新增工作总览：从“自动闭环”到“智能闭环”

## 本页核心内容

- 第一阶段：建模、网格、求解、NSGA-II、SQLite 已形成自动化闭环
- 第二阶段新增：RBF 代理模型 + 稀疏加点 + CCX 验证
- 求解端新增：双齿轮接触、刚体轮毂、位移驱动、失败分类
- 网格端新增：齿根局部加密、`TOOTH_SURF/HUB` 集合生成、接触面校验

### 对应代码模块

- `GearAutoOptManager::startSurrogateAssisted`
- `GearSurrogateModel::train / predict`
- `GearInfillSelector::selectSparseParetoPoints`
- `CCXInpWriter::writeJobInp / renderJobInp`
- `MeshConverter::enrichGearMeshInp`
- `CCXResultParser::parseFrd / parseDat`

### 建议流程图

```text
第一阶段闭环
参数 -> 建模 -> 网格 -> CCX -> 结果 -> NSGA-II

第二阶段增强
历史样本库 -> RBF训练 -> 代理预测 -> 稀疏加点 -> CCX验证 -> 误差反馈
```

### 建议图片

- 左右对比图：左侧“第一次汇报闭环”，右侧“第二阶段智能闭环”
- 右侧用蓝色高亮新增模块

### 建议演讲内容

第二阶段的核心变化是，系统不再只是简单地把每一个设计点都交给有限元求解，而是开始利用已有的 SQLite 历史样本训练 RBF 代理模型。代理模型先在设计空间中进行快速预测和 Pareto 搜索，再选择稀疏、非重复的候选点交给 CalculiX 验证。这样原来的自动化闭环被扩展为“代理预测-真实验证-误差反馈”的智能闭环。同时，在求解和网格两端也进行了工程化增强，以提高批量计算的稳定性。

---

# 3. 新增功能一：RBF代理模型模块

## 本页核心内容

- 已实现独立代理模型类 `GearSurrogateModel`
- 输入变量为 8 个修形与结构参数
- 输出预测 `sigmaMax / uMax / mass`
- RBF 核函数使用高斯核，训练样本不足时拒绝训练

### 对应代码模块

- `src/GearAutoOpt/surrogate/GearSurrogateModel.h`
- `SurrogateSample`
- `SurrogatePrediction`
- `surrogateInputVars`
- `GearSurrogateModel::train`
- `GearSurrogateModel::predict`
- `GearSurrogateModel::solveRbfWeights`
- `GearSurrogateModel::kernel`

### 建议流程图

```text
GearDesignPoint
  -> surrogateInputVars
  -> [x1, x2, ca1, lca1, ca2, lca2, commonWidth, hubRatio]
  -> RBF Model
  -> sigmaMaxPred / uMaxPred / massPred
```

### 建议图片

- 8 维输入到 3 个输出的代理模型结构图
- 小字注释：`GearSurrogateModel.cpp`

### 建议演讲内容

代码中已经实现了独立的 RBF 代理模型模块。代理模型不是直接替代全部有限元计算，而是对已经由 CCX 验证过的样本进行学习。输入变量选取的是齿轮修形和结构相关的 8 个变量，包括两个变位系数、两侧齿顶修形量与修形长度、公共齿宽以及轮毂比。输出包括最大 von Mises 应力、最大位移和质量。模型内部使用高斯 RBF 核函数，并通过线性方程求解 RBF 权重。当有效样本数量低于 `max(20, dim+1)` 时，模型不会强行训练，这一点保证了代理模型不会在样本过少时给出不可靠结果。

---

# 4. 新增功能二：代理辅助NSGA-II优化流程

## 本页核心内容

- 保留原 NSGA-II 算法核心
- 新增 `startSurrogateAssisted` 代理辅助入口
- 历史样本不足时自动运行 LHS + CCX 初始批次
- 每轮代理 Pareto 后选取稀疏点，再用 CCX 验证

### 对应代码模块

- `GearAutoOptManager::startSurrogateAssisted`
- `GearAutoOptManager::evaluatePopulationBySurrogate`
- `GearAutoOptManager::evaluatePopulationByCcx`
- `NSGA2::initLatin`
- `fastNonDominatedSort`
- `evolve`
- `selectNextGen`
- `GearInfillSelector::selectSparseParetoPoints`
- `computeRmae`

### 建议流程图

```text
SQLite验证样本
  -> 样本不足?
      -> LHS初始采样 -> CCX验证
  -> RBF训练
  -> 代理种群初始化
  -> 代理NSGA-II演化
  -> 代理Pareto
  -> 稀疏加点
  -> CCX真实验证
  -> RMAE误差判断
```

### 建议图片

- 闭环流程图，分为“代理域”和“真实有限元域”
- 用虚线表示预测，用实线表示 CCX 验证

### 建议演讲内容

第二阶段没有推翻第一阶段的 NSGA-II，而是在原有算法上增加了代理辅助入口。`startSurrogateAssisted` 会先从 SQLite 中读取同一基准工况下已经由 CCX 验证的样本。如果样本不足，则使用 `initLatin` 进行初始采样，并调用真实 CCX 批量补充样本。随后训练 RBF 模型，在代理模型上运行 NSGA-II，包括非支配排序、SBX 交叉、多项式变异和精英选择。代理得到的 Pareto 解不会直接作为最终结果，而是通过 `GearInfillSelector` 选择离已有样本较远的稀疏点，再返回 CCX 做真实验证。验证后用 `computeRmae` 计算代理误差，用于判断是否继续迭代。

---

# 5. 新增功能三：SQLite样本库与结果复用机制

## 本页核心内容

- 使用 `GearOptResults.db` 保存优化样本
- 区分全局样本库和本次运行库
- 数据库记录 CCX 验证标记、代理使用标记、Pareto 标记
- 代理模型训练数据来自 `verified_by_ccx = 1` 的有效样本

### 对应代码模块

- `src/GearAutoOpt/db/GearOptResultDatabase.h`
- `GearOptResultDatabase::global`
- `GearOptResultDatabase::runSession`
- `openDatabase`
- `insertDesignPointResult`
- `loadValidatedSamples`
- `countValidResults`
- `updateParetoFlag`
- `caseHash / baseCaseHash / designHash`

### 建议流程图

```text
CCX真实计算结果
  -> insertDesignPointResult
  -> GearOptResults.db
      -> 全局库 global()
      -> 运行库 runSession()
  -> loadValidatedSamples
  -> RBF训练样本
```

### 建议图片

- 数据库表结构示意图
- 字段高亮：`sample_source`、`surrogate_used`、`verified_by_ccx`、`isPareto`

### 建议演讲内容

为了支撑代理模型，第二阶段对数据库作用进行了增强。代码中 `GearOptResultDatabase` 同时支持全局库和运行库：全局库用于跨运行复用样本，运行库用于保存本次优化过程。数据库不仅保存设计变量和有限元响应，还包含 `sample_source`、`surrogate_used`、`verified_by_ccx` 等字段。代理模型训练时并不读取所有记录，而是通过 `loadValidatedSamples` 只加载经过 CCX 验证的有效样本。这保证了代理模型的数据来源可追溯，也为论文中“样本复用与代理建模”部分提供了代码依据。

---

# 6. 新增功能四：CalculiX工况写入与求解稳定性优化

## 本页核心内容

- `job.inp` 自动写入材料、边界、载荷、接触和输出请求
- 支持双齿轮 `*RIGID BODY` 轮毂约束
- 支持 `*CONTACT PAIR` 和 `*SURFACE INTERACTION`
- CCX 控制器识别超时、不收敛、奇异矩阵、负雅可比等失败原因

### 对应代码模块

- `CCXInpWriter::writeJobInp`
- `CCXInpWriter::renderJobInp`
- `DualGearRigidBodySpec`
- `BoundarySpec`
- `InpContext`
- `CCXSolverController::start`
- `CCXSolverController::detectCcxPath`
- `failureReasonToString`
- `GearOptCaseRunner::runInpWriteStep`
- `GearOptCaseRunner::runSolveStep`

### 建议流程图

```text
mesh.inp + 工况参数
  -> InpContext
  -> writeJobInp
  -> job.inp
  -> CCXSolverController
  -> ccx_MT.exe
  -> 求解状态分类
```

### 建议图片

- `job.inp` 卡片模块图：Material / Boundary / Rigid Body / Contact / Output
- 失败分类小图标：timeout、no convergence、singular、negative jacobian

### 建议演讲内容

第二阶段对 CalculiX 求解工况进行了较大补充。`CCXInpWriter` 负责根据 `InpContext` 写出 `job.inp`，其中包含材料参数、边界条件、集中载荷、刚体轮毂约束、接触对和输出请求。双齿轮接触工况中，轮毂区域通过 `*RIGID BODY` 与中心参考点关联，齿面通过 `*CONTACT PAIR` 和 `*SURFACE INTERACTION` 建立接触关系。求解端由 `CCXSolverController` 启动 `ccx_MT.exe`，并解析输出信息，把失败原因细分为超时、不收敛、奇异矩阵、负雅可比、崩溃等类型。这些信息会写入结果状态，便于批量优化时定位失败原因。

---

# 7. 新增功能五：Gmsh网格生成、局部加密与集合丰富化

## 本页核心内容

- 自动生成 `gear.geo` 并调用 Gmsh 输出 `mesh.inp`
- 已实现齿根曲线识别与 `Distance + Threshold` 局部加密
- 后处理生成 `GEAR1/GEAR2_HUB`、`TOOTH_OUTER`、`TOOTH_SURF`
- 接触面 master/slave 集合用于 CalculiX 接触对

### 对应代码模块

- `GearOptCaseRunner::runMeshStep`
- `writeGearGeoFile`
- `writeGearGeoContent`
- `collectRootCurveTagsForGmsh`
- `collectZCurveTagsForGmsh`
- `MeshConverter::stripSurfaceElements`
- `MeshConverter::enrichGearMeshInp`
- `MeshConverter::collectToothNodesFromSolidExteriorFaces`
- `MeshConverter::countSurfaceFaces`

### 建议流程图

```text
gear1.brep / gear2.brep
  -> gear.geo
  -> Gmsh -3 -format inp
  -> mesh.inp
  -> stripSurfaceElements
  -> enrichGearMeshInp
  -> HUB / TOOTH_OUTER / TOOTH_SURF / master-slave
```

### 建议图片

- 齿根局部加密示意图
- `gear.geo -> mesh.inp -> enriched mesh.inp` 三段式流程图

### 建议演讲内容

网格部分的新增重点有两个。第一是 `runMeshStep` 不只是简单调用 Gmsh，而是会先导出 BREP，再写出 `gear.geo` 文件，并在其中加入齿根局部加密设置。代码中可以看到 `Distance + Threshold` 的加密逻辑，以及 `Mesh.MeshSizeMin` 和 `Mesh.MeshSizeMax` 的设置。第二是对 Gmsh 输出的 `mesh.inp` 进行后处理。`MeshConverter` 会清理不适合 CCX 的面单元，并根据齿轮几何位置生成轮毂、齿面外周、局部接触齿面等集合。这些集合是后续刚体约束、载荷施加和接触分析的基础。

---

# 8. 新增功能六：有限元结果解析、可视化与误差评价

## 本页核心内容

- 优先解析 `job.frd` 中的位移和应力结果
- 缺失时回退解析 `job.dat`
- 支持按节点/集合提取最大 von Mises 和最大位移
- 支持导出 `result.vtu` 用于 ParaView 后处理
- 已实现 RMAE 代理误差指标和接触面法向校验

### 对应代码模块

- `CCXResultParser::parseFrd`
- `CCXResultParser::parseDat`
- `parseDatDisplacements`
- `maxVonMisesForNodes`
- `maxDisplMagnitudeForNodes`
- `vonMisesPercentileForNodes`
- `CalculiXResultVtkExport::exportCalculixResultToVTK`
- `MeshConverter::verifyCcxElementSurfaceFaces`
- `GearSurrogateMetrics::computeRmae`
- `GearOptCaseRunner::runParseStep`

### 建议流程图

```text
CCX输出
  -> job.frd
      -> 位移U / 应力S / max von Mises
  -> job.dat
      -> 回退应力解析
  -> result.vtu
      -> ParaView可视化
  -> RMAE / SurfaceVerify
      -> 误差与可信度支撑
```

### 建议图片

- FRD/DAT 双路径解析图
- 右侧放 `result.vtu` 云图占位图
- 底部小字：代码未发现完整 Hertz/AGMA 理论公式模块

### 建议演讲内容

有限元结果部分已经形成较完整的解析链路。`runParseStep` 中优先读取 `job.frd`，从中提取节点位移和应力结果，计算最大位移和最大 von Mises 应力。如果 FRD 中数据缺失，则回退到 `job.dat` 解析积分点应力。可视化方面，`CalculiXResultVtkExport` 会把网格和结果导出为 `result.vtu`，用于 ParaView 查看云图。误差评价方面，代码已经实现代理模型预测误差 RMAE，同时对接触面法向进行校验。但需要注意，当前代码中没有发现完整的 Hertz 接触应力或 AGMA 齿根强度公式校核模块，因此论文中应把这一部分表述为“数值结果解析与误差评价支撑”，后续再补充理论公式对比。

---

# 9. 代码完成度评估：已完成、半完成与待补充

## 本页核心内容

- 已完成：RBF 训练预测、代理加点、SQLite 样本读取、CCX 工况、Gmsh 加密、结果解析
- 工程化较完整：失败分类、数据库复用、VTU 导出、GUI 参数入口
- 待完善：理论公式验证、代理模型交叉验证图表、更多算例对比
- 不宜夸大：代码中未见完整论文自动生成模块

### 对应代码模块

- 已完成：`GearSurrogateModel`、`GearInfillSelector`、`GearOptResultDatabase`
- 已完成：`CCXInpWriter`、`CCXSolverController`、`MeshConverter`
- 已完成：`CCXResultParser`、`CalculiXResultVtkExport`
- 待补充对应：理论公式验证模块尚未发现

### 建议流程图

```text
完成度矩阵
功能实现 | 工程稳定 | 论文可写 | 后续待补
RBF      | 中       | 高       | 验证曲线
CCX      | 高       | 高       | 参数对比
Gmsh     | 高       | 高       | 网格收敛
验证     | 中       | 中       | 理论公式
```

### 建议图片

- 三列看板：已完成 / 已具备基础 / 待补充
- 每项用小模块卡片，不使用长段落

### 建议演讲内容

从代码完成度看，第二阶段新增模块已经不只是设计方案，而是有明确的实现文件和函数入口。RBF 代理模型、稀疏加点、SQLite 样本读取、CCX 工况写入、Gmsh 局部加密和结果解析都已经有可追踪的代码。工程化方面，失败分类、数据库复用和 VTU 导出也已经具备。当前还需要谨慎处理的是有限元理论验证部分：代码中已经有数值结果解析、RMAE 误差和接触面校验，但还没有完整的理论公式校核模块。因此论文和答辩中应把已完成部分和计划补充部分区分清楚。

---

# 10. 论文编写进展：代码模块到论文章节的映射

## 本页核心内容

- 论文主体可围绕 GearAutoOpt 新增模块展开
- 第二阶段代码可支撑“代理模型”“求解工况”“网格加密”“误差评价”章节
- `docs/结构说明.md` 可作为系统结构说明基础
- 后续论文重点：实验数据、误差曲线、网格收敛与理论验证补充

### 对应代码模块

- 第三章系统设计：`GearDesignPoint`、`GearOptConfig`、`GearAutoOptManager`
- 第四章代理优化：`GearSurrogateModel`、`GearInfillSelector`、`NSGA2`
- 第五章有限元自动求解：`GearOptCaseRunner`、`CCXInpWriter`、`CCXSolverController`
- 第六章结果与验证：`CCXResultParser`、`CalculiXResultVtkExport`、`computeRmae`
- 附录/工程实现：`GearOptResultDatabase`、`MeshConverter`

### 建议流程图

```text
代码模块
  -> 论文章节
  -> 实验图表
  -> 答辩PPT
```

### 建议图片

- 左侧代码目录树，右侧论文章节树，中间用连线映射

### 建议演讲内容

论文编写方面，目前最适合的组织方式是从代码模块映射到论文章节，而不是重复介绍 FastCAE 平台本身。系统设计章节可以对应 `GearDesignPoint`、`GearOptConfig` 和 `GearAutoOptManager`；代理优化章节对应 RBF 模型、稀疏加点和 NSGA-II；有限元自动求解章节对应单工况流水线、CCX 输入文件和求解控制；结果验证章节对应 FRD/DAT 解析、VTU 可视化、RMAE 误差以及接触面校验。后续论文工作的重点是补充实验数据图表，尤其是代理误差曲线、网格收敛分析和理论公式对比。

---

# 11. 当前问题与下一步计划

## 本页核心内容

- 当前问题 1：代理模型需要更多 CCX 验证样本支撑
- 当前问题 2：理论验证公式尚未形成独立代码模块
- 当前问题 3：网格收敛和求解参数对比数据还需系统整理
- 下一步：补充实验批次、导出云图与曲线、完善论文验证章节

### 对应代码模块

- 样本补充：`GearAutoOptManager::startSurrogateAssisted`
- 误差评价：`computeRmae`
- 网格收敛数据来源：`GearDesignPoint::meshSize_mm`、`nodeCount`、`elementCount`
- 求解失败统计：`CCXFailureReason`
- 结果可视化：`exportCalculixResultToVTK`

### 建议流程图

```text
下一步实验计划
样本扩充 -> 代理误差曲线 -> 网格收敛 -> 理论公式对比 -> 论文定稿
```

### 建议图片

- 时间线图：代码完善、实验计算、论文图表、论文定稿
- 右侧放“待生成图表清单”

### 建议演讲内容

下一步工作主要集中在实验和论文验证上。代码层面，代理模型和 CCX 自动求解链路已经具备基础能力，但代理模型的可信度还依赖更多真实 CCX 样本。因此需要继续运行样本扩充批次，绘制 RMAE 随样本数量变化的曲线。同时，网格部分已经记录了网格尺寸、节点数和单元数，可以进一步组织成网格收敛分析。理论验证方面，目前代码中还没有完整的 Hertz 或 AGMA 公式模块，后续需要补充手算或脚本对比结果，并在论文中与有限元结果进行误差分析。

---

# 12. 阶段性总结：第二阶段形成的可交付成果

## 本页核心内容

- 形成代理辅助优化框架：RBF + NSGA-II + CCX 验证
- 完善双齿轮接触求解工况：刚体轮毂、接触对、位移驱动、失败分类
- 完善网格与后处理：齿根加密、集合丰富化、接触面校验
- 完善结果链路：FRD/DAT 解析、RMAE、VTU 导出
- 论文进入“代码模块-实验图表-章节映射”阶段

### 对应代码模块

- `src/GearAutoOpt/surrogate/`
- `src/GearAutoOpt/opt/NSGA2.h/cpp`
- `src/GearAutoOpt/runner/GearAutoOptManager.cpp`
- `src/GearAutoOpt/runner/GearOptCaseRunner.cpp`
- `src/GearAutoOpt/solver/`
- `src/GearAutoOpt/db/GearOptResultDatabase.cpp`

### 建议流程图

```text
第二阶段成果
代理模型
  + 求解工况
  + 网格加密
  + 结果验证
  + 论文映射
  -> 毕设最终系统与论文基础
```

### 建议图片

- 五边形成果图：代理、求解、网格、验证、论文
- 底部代码路径小字作为证据锚点

### 建议演讲内容

总体来看，第二阶段已经把平台从“能自动跑通”推进到“能利用历史样本进行智能优化”。RBF 代理模型、NSGA-II、SQLite 样本库和 CCX 验证已经形成闭环；CalculiX 工况写入和求解控制更适合双齿轮接触分析；Gmsh 网格生成增加了齿根局部加密和接触面集合后处理；结果部分具备 FRD/DAT 解析、VTU 导出和代理误差评价。下一阶段的重点不再是大规模搭框架，而是补充实验数据、完善理论验证，并把代码成果转化为论文中的图表和论证。

