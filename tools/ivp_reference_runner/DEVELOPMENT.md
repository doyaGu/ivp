# ivp_reference_runner 开发说明

本文档说明 `tools/ivp_reference_runner` 的模块边界、扩展方式与验证流程。

## 1. 目标

`ivp_reference_runner` 用于在 legacy IVP 上生成可重复的 JSONL 轨迹，供 `ivp-c17` 做场景逐帧对齐。

## 2. 模块结构

### 2.1 入口与公共层

- `main.cxx`
  - 仅负责流程编排：解析参数、初始化环境、调度场景、采样输出、释放资源。
- `ref_runner.hxx`
  - 全局共享声明：枚举、配置结构、辅助函数声明、场景分发声明。
- `ref_runner_common.cxx`
  - 纯通用能力：参数解析、输出写入、对象构造 helper、通用约束/执行器 helper。

### 2.2 场景层（按职责拆分）

- `ref_runner_scenarios_basic.cxx`
  - 基础刚体/碰撞类场景：如 `freefall`、`two_balls`、`cubes`、`vehicle`、`collision_filter` 等。
- `ref_runner_scenarios_constraints.cxx`
  - 约束主导场景：如 `springs`、`rope`、`motor`、`stiff_spring`、`check_distance`。
- `ref_runner_scenarios_controllers.cxx`
  - 控制器/体积力/特殊交互：如 `buoyancy`、`force_actuator`、`forcefield`、`motion_controller`、`phantom`。
- `ref_runner_scenarios_car.cxx`
  - 实轮车辆：`car_real_wheels`（`IVP_Car_System_Real_Wheels`，通过 `ScenarioResources::step_hook` 在指定步施加油门/转向/手刹，`cleanup_hook` 负责删除车辆）。
- `ref_runner_constraint_car_fixed.cxx`
  - `ivp_controller/ivp_constraint_car.cxx` 的覆盖副本，仅修正一行：`IVP_Constraint_Solver_Car_Builder` 的临时矩阵 `aligned_row_len` 为 0（`P_MEM_CLEAR`），`set_value()` 把所有行写进第 0 行，求逆作用于未初始化内存，原库中实轮车辆无法创建（"failed to initialize constraint system"）。链接时该文件定义了整个翻译单元的符号，因此不会再从 `libivp_controller.a` 取出原始 `ivp_constraint_car.o`。ivp-c17 的移植（`src/controllers/ivp_constraint_car.c`）含相同修正。
- `ref_runner_scenarios_merge.cxx`
  - 共享 core：`merge_objects`（`IVP_Environment::merge_objects`）、`object_attach`（`IVP_Object_Attach`）、`merge_buoyancy`（浮力作用于合并 core 的每个对象）。只在原版安全的输入上合并（未模拟、无接触点的对象：模拟中的 core 合并后 `q_world_f_core_next_psi` 为零，`fast_normize_quat` 死循环；摩擦系统会保留被删 core 的指针）。
  - `libivp_physics.a` 中的 `ivp_object_attach.cxx.o` 调用 `IVP_Core::inline_calc_at_position` / `inline_calc_at_quaternion` 和 `IVP_Hull_Manager::check_hull_synapses`，却没有包含它们的 inline 定义（`ivp_core_macros.hxx`、`ivp_hull_manager_macros.hxx`），单独无法链接；本文件取这三个原版 inline 函数的地址，生成它们的非 inline 副本（`ref_emit_*`）。
- `ref_runner_scenarios_dispatch.cxx`
  - 统一分发，仅按顺序调用：`basic -> constraints -> controllers -> car`。

## 3. 场景命名约定

- CLI 名称与 `ivp-c17/examples` 对齐。
- 兼容别名保留在 `parse_scenario()`：
  - `balls -> two_balls`
  - `friction -> slope_friction`
- 新增别名时，必须保证映射为已存在场景枚举。

## 4. 新增场景标准流程

1. 在 `ref_runner.hxx` 的 `Scenario` 枚举增加新值。
2. 在 `parse_scenario()` 增加名字映射（必要时加兼容别名）。
3. 按职责把 setup 函数写入以下之一：
   - basic / constraints / controllers。
4. 在对应模块的 `setup_*_scenarios()` 中接入分支并返回 `true`。
5. 不要在 `main.cxx` 直接写场景逻辑。
6. 如需新增可复用 helper，放入 `ref_runner_common.cxx` 并在头文件声明。

## 5. 稳定性与可重复性要求

- 优先保证 deterministic（固定步长 + 固定初始条件）。
- 避免过激参数导致 legacy 断言或发散（极大力、过硬参数、过长仿真）。
- 所有动态体创建后统一 `wake_and_enable()`，避免遗漏导致状态不一致。

## 6. 验证清单（Checklist）

- [ ] 父级构建系统已实际声明 `ivp_reference_runner` 目标，或已有可运行的二进制。
- [ ] `--scenario <name>` 可识别新增场景名称（未知名称会报 `Unknown scenario`）。
- [ ] 新增场景可独立运行并生成 JSONL。
- [ ] 输出包含稳定的对象数量与类型标签。
- [ ] 不引入对旧模块（physics/examples）的依赖。
- [ ] 若用于对齐，已把新的 JSONL 输出保存到调用方约定的基线路径（本仓库当前未自带 `tools/testdata/` 目录）。

## 7. 推荐命令

在仓库根目录执行前，请先确认两件事：

1. 当前父级 CMake/构建脚本确实声明了 `ivp_reference_runner` 目标。
2. 输出路径由调用方自己提供；不要假定仓库内存在 `tools/testdata/`。

如果父级构建已经接入该目标，可以用下面的形式：

```powershell
cmake --build build --target ivp_reference_runner
.\build\ivp_reference_runner.exe --scenario freefall --steps 120 --dt 0.0166667 > .\tmp\ref_freefall.jsonl
```

如果仓库本身还没有把该目标接入 CMake，上面的 `--target ivp_reference_runner` 会失败；这属于当前仓库布局限制，不是命令参数问题。

如果要批量更新基线，优先使用现有批处理/脚本入口，避免手工命令参数不一致。
