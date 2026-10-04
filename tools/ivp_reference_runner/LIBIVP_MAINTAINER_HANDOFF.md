# libivp 维护交接：IVP reference 更新至 `8b81109`

本文供维护 C17 端 `libivp` 的 agent 使用。目标是让 `libivp` 以新的
reference 为准重新验证行为，同时保留 C17 API、模块边界和已有扩展。

## 基线

- reference 仓库：`doyaGu/ivp`
- 旧基线：`7579664`（旧 `origin/master`）
- 新基线：`8b81109`
- 比较范围：`7579664..8b81109`，共 51 个提交
- 可直接取得新基线的远端分支：`libivp-parity-runner`
- 本地 `ivp/master` 以 `8b81109` 为引擎基线，并在其上保存本交接文档；本文
  编写时尚未推送该主分支
- 对照的 `libivp` 状态：`411752d`（`main`，本地领先远端 4 个提交）

不要把 C++ 补丁逐行翻译到 C。先把每项改动还原成行为契约，再检查
`libivp` 是否已经以不同结构实现了同一契约。

## 执行顺序

1. 将 reference 固定到 `8b81109`，记录 `git rev-parse HEAD` 的结果。
2. 在修改 `libivp` 前运行一次 Release 和 Debug parity，保存每个场景的
   第一个差异、非 JSON 文本差异和 Debug 断言序列差异。
3. 先处理下文“优先核对”中的五项，再审计已经对齐的行为和其余防御性
   修复。每项代码变更都要有一个能在旧实现上失败的回归测试。
4. 用新 reference 重新生成 JSONL。参考数据只能来自固定到 `8b81109`
   的 IVP runner。
5. 更新 `docs/PARITY_GAP_INVENTORY.md` 和 `docs/PORT_WORK_REPORT.md`：把已经
   修入 reference 的项目从“有意偏离”改成双方共同契约。
6. 运行完整 Release/Debug parity、单元测试和 sanitizer。完成标准见文末。

建议先运行：

```sh
cd ../libivp
IVP_REF_SRC=../ivp tools/parity/check.sh --release --debug
tools/regen_reference.sh ../ivp
```

`tools/regen_reference.sh` 会自行编译
`../ivp/tools/ivp_reference_runner/*.cxx`；reference 根 CMake 目前没有注册
runner target，这不影响 `libivp` 的脚本。

runner 的场景实现位于 reference 仓库内，对 `libivp` 没有硬依赖。`main.cxx`
仍保留一个开发期兼容入口：如果 runner 二进制旁存在 `test_scenarios`，会委托它
生成输出。`libivp` 的 regeneration/parity 脚本必须继续使用隔离目录并拒绝这种
布局，避免把 C17 输出误当成 reference 数据。

## 优先核对：当前 libivp 源码仍可能保留旧行为

以下判断基于 `libivp` 的 `411752d` 工作树，应先验证再修改。

| reference 提交 | 新契约 | `libivp` 当前观察与动作 |
|---|---|---|
| `c90899a` | 延迟删除队列在 drain 期间仍保持延迟；删除回调再次删除当前对象为 no-op；回调请求删除另一个对象时将其追加到同一队列 | `src/dynamics/ivp_environment.c` 只有 depth 和队列。为环境状态增加 draining 和 object-being-deleted；`destroy_object_by_handle` 在 depth 或 draining 时入队，并识别正在析构的对象。移植 `tests/core/deferred_deletion_reentrancy.cxx` 的场景。 |
| `cf18246` | `copy_to_sub_matrix` 的外部源缓冲区是 packed `columns × columns`，源行步长为 `columns`；目标仍使用自己的 `aligned_row_len` | `src/solver/ivp_great_matrix.c:1133` 当前用 `m->aligned_row_len` 读取源。按 public packed contract 改为 `m->columns`，并移植 `tests/core/matrix_copy_contract.cxx`。 |
| `8a9c33e` | matrix cache index 同时夹到 `[0, IVP_3D_SOLVER_MAX_STEPS_PER_PSI]`；只对原本就在范围内的请求检查 time/index 一致性 | `src/collision/ivp_mindist_event.c` 当前只处理上界，负 index 仍可越界。补下界并增加正、负越界测试。 |
| `063fceb`, `e0dd03a` | compact-grid point offset 的真实要求是 4 字节对齐；实际写入尺寸可以小于等于预留的 padded `byte_size` | `src/builder/ivp_gridbuild_array.c` 仍保留 16 字节和严格相等的 NONFATAL 断言。改成 `(offset & 3) == 0` 与 `used <= byte_size`，同步更新 parity 文档和 Debug 断言预期。 |
| `1587518` | 先把无效或零惯量分量夹到有限正值，再执行长度断言并计算逆惯量 | `src/dynamics/ivp_core.c` 当前在 clamp 前发 NONFATAL 断言。移动断言或改写测试，使零惯量输入得到有限正惯量且不产生旧的 Debug 断言差异。 |

延迟删除仍应保持每个 environment 自己的状态；reference 使用进程级静态状态是
原 C++ 结构的限制，不是 C17 端需要照搬的设计。

## 已成为 reference 正式行为的缺陷修复

这些项目已经在 `libivp` 文档中列为有意偏离。现在 reference 本身也采用了
同一方向，维护工作以重新验证、补测试和改文档为主。

| reference 提交 | 行为契约 | `libivp` 重点 |
|---|---|---|
| `4d818ea` | PSI 重置时间基准时，从队列事件减去 `time_of_last_psi - base_time`，保持绝对到期时间 | 检查 `ivp_time_manager_rebase_for_psi`；加入跨多个 PSI 的用户事件测试。 |
| `81357f3` | car constraint 临时矩阵清零后显式设置 `aligned_row_len = columns` | C17 端已有实轮车辆路径；确认 runner 不再需要把这一项描述成 reference override。 |
| `8ad306e` | `merge_objects` 对空、NULL、外部环境对象、共享 core、活动 core、重复对象、接触中的对象原子式拒绝；合法的 7 个以上对象可合并 | 保留“失败时对象和 core 完全不变”的契约，覆盖所有拒绝分支和大于 6 个对象的成功分支。 |
| `f105953` | `attach_object(NULL, ...)`、`attach_object(..., NULL)` 和 self-attach 都直接返回 | 保留父子对象、core 和引用计数不变。 |
| `6623a1d` | sphere query 先识别真正的 polygon manager；其它 polygon-like manager 走虚拟 ledge 查询，grid 可安全查询 | C17 vtable 路径已接近该设计；增加 compact-grid 内外两个查询断言。 |
| `8ca7bb2` | merged core 从 `q_world_f_core_last_psi` 初始化 next-PSI quaternion；临时 core 尚无对象时跳过 hull prefetch | 检查 merged core 直接构造和正常 merge 两条路径。 |

`ref_runner_constraint_car_fixed.cxx` 仍在 runner 源码中，并继续提供完整翻译单元
覆盖。新 reference 库已经包含同一行修复，因此该 override 目前是冗余的，而不是
新的行为来源。清理它属于后续 runner 维护，不应阻塞 `libivp` 对齐。

## 其它引擎修复

`9d9c3f1..3a1a39a` 还包含一组未改变合法输入语义的安全性和可移植性修复。
审计 C17 对应路径，不需要照搬仅适用于 C++ 对象生命周期的写法。

- 容器：缺失元素删除为 release no-op；边界、容量和索引在写入前检查；
  unique insertion 复用普通插入路径。
- 数值：退化 QR 输出确定化；NaN 或超范围 PSI interval 不修改环境；负的
  rounding residual energy 夹到 0；随机种子用 unsigned 定义回绕；负 raster
  坐标不再左移 signed integer。
- 初始化：core、object template、debug manager、ledge soup 和 polygon helper
  不再用 raw memset 覆盖含构造对象的表示。
- 矩阵：内部 matrix-to-matrix copy 使用各自的 aligned stride；导出到外部缓冲区
  使用 packed stride。
- compact grid：按真实对象布局计算 flexible header offset，并 placement-construct
  分配出的对象。
- 调试输出：pointer low bits 先转成与 `%u` 匹配的 unsigned 类型。
- 构建：公开头保持 C++98；这只修复 reference 的 C++ 头，不改变 C17 API。

其中 unsigned random wrap、invalid PSI、energy clamp 和 raster scaling 在当前
`libivp` 源码中已经能看到等价实现。其余项目应以 sanitizer 和现有容器/矩阵测试
为证据判定，不要仅凭结构相似标记完成。

## Runner 与场景

reference 仓库现在自带完整 parity runner，场景按职责拆分在：

- `ref_runner_scenarios_basic.cxx`
- `ref_runner_scenarios_constraints.cxx`
- `ref_runner_scenarios_controllers*.cxx`
- `ref_runner_scenarios_car.cxx`
- `ref_runner_scenarios_geometry.cxx`
- `ref_runner_scenarios_misc.cxx`
- `ref_runner_scenarios_merge.cxx`

`libivp/tests/test_scenarios.c` 已有对应注释和场景。同步时以
`tools/scenarios.txt` 为场景清单的单一来源，确保名称、dt、steps、对象标签和采样
顺序完全一致。runner 的详细边界见同目录的 `DEVELOPMENT.md`。

新的 reference 核心回归位于 `tests/core/`，由
`-DIVP_BUILD_TESTS=ON` 显式启用，默认关闭。`legacy_defects.cxx` 提供八个模式：

- `time_event`
- `car_builder`
- `merge_duplicate`
- `attach_self`
- `matrix_cache`
- `sphere_grid`
- `merged_core`
- `zero_inertia`

另有独立的 deferred deletion 和 matrix copy contract 测试。C17 端应把这些行为
纳入现有模块测试，而不是依赖 C++ 测试二进制。

## Examples 与依赖边界

reference 新增了 20 个 quickstart、8 个 advanced example、共享 sample app、SDL3
renderer、Nuklear UI 和 bundled glad。它们不是 `libivp` 引擎 parity 的组成部分，
对 `ivp-c17`/`libivp` 没有源码或构建依赖，也不应改变 `libivp` 正常库构建的
依赖。

reference 的边界如下：

- `IVP_BUILD_EXAMPLES=OFF`，默认不配置 examples。
- `IVP_BUILD_TESTS=OFF`，默认不构建核心回归。
- `IVP_BUILD_EXAMPLE_TESTS` 只在 examples 开启时存在，默认关闭。
- 普通库构建只需要原来的 C++98 工具链，不需要 SDL3、Nuklear、OpenGL 或 Python。
- examples 开启后需要 CMake 3.20、C99、C++98、Nuklear header 和 OpenGL 3.3。
- SDL3 优先使用已安装的 CMake package，否则 FetchContent 固定到
  `release-3.2.0`。
- glad 0.1.36 的 OpenGL 3.3 core loader 已随 reference 源码保存。
- examples 和 renderer 不进入 engine package exports。
- headless smoke 只验证启动和输出分类；dummy video 无法创建 OpenGL context 时
  记为 skip，不代表渲染正确。

`libivp` 已有自己的 SDL3 GPU renderer 和 examples。把 reference examples 当作场景
与交互行为的补充证据即可；不要用 C++ sample framework 替换 C17 的 render 模块。

## reference 侧验证结果

在 `8b81109` 上已经完成：

- Release library-only clean build，tests/examples 均关闭。
- Debug core regression：10/10。
- ASan Debug core regression：10/10。
- 显式 `-DDEBUG` core regression：10/10。
- optional examples regression：3/3。
- `git diff --check`。

这些结果证明 reference 补丁自身可构建并通过定向回归，不替代 `libivp` 的逐位
parity、C17 sanitizer 和平台验证。

## 完成标准

维护任务仅在以下条件全部满足后完成：

- [ ] reference 被固定到 `8b81109`，日志记录该 SHA。
- [ ] 上述五个优先项逐一审计，并有旧实现会失败的 C17 回归测试。
- [ ] 八个 legacy defect 行为在 C17 测试中都有明确覆盖或已有等价测试引用。
- [ ] Release parity 的 JSONL、stdout 非 JSON 文本和 stderr 全部匹配。
- [ ] Debug parity 的 JSONL、调试文本和断言序列全部匹配。
- [ ] `ctest`、ASan 和 UBSan 通过。
- [ ] 默认 `IVP_BUILD_EXAMPLES=OFF` 的库构建不获取图形依赖。
- [ ] `PARITY_GAP_INVENTORY.md` 与 `PORT_WORK_REPORT.md` 不再把已修入 reference
      的行为描述成 reference 缺陷。
- [ ] 每个 `libivp` 提交保持单一主题，并遵守 `AGENTS.md` 的提交格式。
