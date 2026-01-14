[English](README.md) | **中文**

# EstimatorFrame — 估计算法接口与多语言调用示例

> 面向 **Windows / Linux** 的统一工程，演示将估计器核心（C/C++）编译为共享库，并由 **C / C++ / Python / MATLAB** 调用。文档遵循清晰目录约定与一键化构建（VS Code Tasks）。

---

## 1) 目录与产物约定

```
EstimatorFrame/           # 工程根
├─ Estimator/             # 共享库生成代码与头文件
│  ├─ EstimatorPortN.h
│  ├─ EstimatorPortN_MatlabDLL.cpp             (MATLAB<->DLL 中间层, Windows)
│  └─ EstimatorPortN_MatlabSO.cpp              (MATLAB<->SO  中间层, Linux)
├─ Output/                # 可执行文件产出与共享库（*.dll / *.lib / *.so / *.mexa64o）
│  ├─ EstimatorPortN.dll / EstimatorPortN.lib   (Windows)
│  ├─ EstimatorPortN.so                        (Linux)
├─ Temp/                  # 中间产物（build 目录、编译临时文件等）
├─ ObservationData/       # 观测数据（默认示例）
├─ EstimationResult/      # 估计结果（*.txt，带时间戳）
└─ *.c / *.cpp / *.py / *.m
```

> **运行位置**：建议始终在工程根目录启动终端（或从 VS Code 的工作区根执行任务）。

---

## 2) 环境准备

### Windows

* Microsoft Visual Studio **Build Tools 2022**（MSVC、NMake）
* **CMake ≥ 3.16**
* **Python 3.x**（可选，仅用于 Python 演示）
* **MATLAB R2020a+**（可选，仅用于 MATLAB 演示）
* 将 VS Build Tools 提供的开发环境脚本（`vcvars64.bat`）用于任务初始化（Tasks 已内置）。

### Linux (Ubuntu 20.04+ 测试通过)

* **gcc/g++**、**make**
* **CMake ≥ 3.16**
* **Python 3.x**（可选）
* **MATLAB R2020a+**（可选）

---

## 3) 快速开始（推荐：VS Code 任务）

按 **Ctrl + Shift + B**，在弹出的列表中选择：

* **Build EstimatorPortN (Windows/Linux)**：编译共享库 → `Output/EstimatorPortN.dll`（+`EstimatorPortN.lib`）或 `Output/EstimatorPortN.so`
* **Build C_Demo / Cpp_Demo**：直接将估计器源码编入可执行 → `Output/C_Demo` / `Output/Cpp_Demo`
* **Build C_DLL_Demo / Cpp_DLL_Demo（Windows）**：运行时加载 `Estimator/EstimatorPortN.dll` → `Output/*.exe`
* **Build C_SO_Demo / Cpp_SO_Demo（Linux）**：运行时加载 `Estimator/EstimatorPortN.so` → `Output/*`

> **提示**：任务已设置好生成目录与依赖，无需手动敲命令。若你更偏爱命令行，可参考 `CMakeLists.txt` 的目标与输出路径（本文不再赘述）。

---

## 4) 测试数据

* 默认数据：`ObservationData/DoubleReflectorTrace/Trace1000.txt`
* 可替换为你自己的观测数据（注意保持格式/维度一致）。

---

## 5) C 程序（直接链接）

1. **构建**：选择 `Build C_Demo (Windows/Linux)`
2. **运行**：

   * Windows：`Output\C_Demo.exe`
   * Linux：`./Output/C_Demo`
3. **预期输出**（示例）：

   ```
   Tested Estimator v0.00 is initialized
   Unscented Kalman estimator is initialized
   EstimatorPort terminated.
   StateSpaceModel1_ finished...
   Estimation data has been written to EstimationResult/EstimationResult_YYYYMMDDhhmmss.txt
   Program terminated...
   ```
4. 结果文件位于 `EstimationResult/`。

---

## 6) C++ 程序（直接链接）

1. **构建**：选择 `Build Cpp_Demo (Windows/Linux)`
2. **运行**：

   * Windows：`Output\Cpp_Demo.exe`
   * Linux：`./Output/Cpp_Demo`
3. 查看 `EstimationResult/` 下的结果文件。

---

## 7) 生成共享库 & 运行时加载示例

### 7.1 生成共享库

* Windows：`Build EstimatorPortN (Windows, DLL -> Output/)` → 生成 `Output/EstimatorPortN.dll` 与 `Output/EstimatorPortN.lib`
* Linux：`Build EstimatorPortN (Linux, SO -> Output/)` → 生成 `Output/EstimatorPortN.so`

### 7.2 运行时加载（Windows：DLL，Linux：SO）

* **Windows**：

  * 选择 `Build C_DLL_Demo` / `Build Cpp_DLL_Demo`
  * 运行 `Output\C_DLL_Demo.exe` / `Output\Cpp_DLL_Demo.exe`
  * **要求**：运行时 `Output/` 需在 `PATH` 中，或从工程根运行以便找到同相对路径的 `Output\EstimatorPortN.dll`
* **Linux**：

  * 选择 `Build C_SO_Demo` / `Build Cpp_SO_Demo`
  * 运行 `./Output/C_SO_Demo` / `./Output/Cpp_SO_Demo`
  * 若系统未设置 rpath，可临时：`export LD_LIBRARY_PATH="$PWD/Output:$LD_LIBRARY_PATH"`

---

## 8) Python 调用共享库

### 8.1 Windows：`Python_DLL_Demo.py`

* 在工程根执行：`python Python_DLL_Demo.py`
* 要求：`Output/EstimatorPortN.dll` 可被加载（从工程根运行或将 `Output/` 加入 `PATH`）。

### 8.2 Linux：`Python_SO_Demo.py`

* 在工程根执行：`python3 Python_SO_Demo.py`
* 要求：`Output/EstimatorPortN.so` 可被加载（从工程根运行或设置 `LD_LIBRARY_PATH`，如上）。

> Python 也可通过 `subprocess.run([...])` 直接调用 `Output/*` 可执行；共享库方式更高效且跨语言一致。

---

## 9) MATLAB 调用（Windows，DLL）

推荐将 **一次性编译** 嵌入到 M 文件，首次运行自动生成 MEX，之后无需重复编译。

```matlab
% ===== Windows + DLL：一次性 MEX 编译（若未生成） =====
projRoot = fileparts(mfilename('fullpath'));
estDir   = fullfile(projRoot, 'Estimator');
outDir   = fullfile(projRoot, 'Output');
srcCpp   = fullfile(estDir, 'EstimatorPortN_MatlabDLL.cpp');
outBase  = fullfile(outDir, 'EstimatorPortN_MatlabDLL');  % 不带扩展名
outMex   = [outBase '.' mexext];                          % 期望为 .mexw64

if ~exist(outMex, 'file')
    % 仅需第一次：可手动先运行一次  mex -setup C++
    mex('-v', ['-I' estDir], srcCpp, fullfile(outDir, 'EstimatorPortN.lib'), ...
        '-output', outBase);
end

% 后续你的 M 代码即可直接调用：EstimatorPortN_MatlabDLL('initialize'|'estimate'|'terminate')
```

> **说明**：Windows 下建议直接传入 `.lib` 的完整路径（见 `fullfile(estDir,'EstimatorPortN.lib')`）。MEX 产物将放在 `Estimator/`，与 `EstimatorPortN.dll` 同目录，便于运行时加载。

---

## 10) MATLAB 调用（Linux，SO）

同样推荐一次性自动编译并设置 rpath 以便运行时就近找到 `.so`：

```matlab
% ===== Linux + SO：一次性 MEX 编译（若未生成） =====
projRoot = fileparts(mfilename('fullpath'));
estDir   = fullfile(projRoot, 'Estimator');
outDir   = fullfile(projRoot, 'Output');
srcCpp   = fullfile(estDir, 'EstimatorPortN_MatlabSO.cpp');
outBase  = fullfile(outDir, 'EstimatorPortN_MatlabSO');   % 不带扩展名
outMex   = [outBase '.' mexext];                          % 期望为 .mexa64

if ~exist(outMex, 'file')
    % 仅需第一次：可手动先运行一次  mex -setup C++
    mex('-v', ['-I' estDir], ...                      % 头文件路径
        srcCpp, ...                                   % 源文件
        ['-L' outDir], '-lEstimatorPortN', ...        % 链接 EstimatorPortN.so
        "LDFLAGS=$LDFLAGS -Wl,-rpath,'$ORIGIN'", ... % 运行时在 MEX 同目录找 .so
        '-output', outBase);
end

% 后续你的 M 代码即可直接调用：EstimatorPortN_MatlabSO('initialize'|'estimate'|'terminate')
```

> **若你的 CMake 改回了默认前缀**（生成 `libEstimatorPortN.so`），仍可用 `-lEstimatorPortN` 链接，ld 会自动剥离 `lib` 前缀与 `.so` 后缀。

---

## 11) 测试 MATLAB 程序（纯 .m）

1. 打开 `M_Demo.m` 并运行。该脚本调用接口 `Estimator/M_Estimators/StateSpaceModel1.m`，其内部使用 `Estimator/M_Estimators/Estimator3001.m` 进行估计。
2. 运行结束后，在 `EstimationResult/` 查看生成的 `EstimationResult_YYYYMMDDhhmmss.txt`。

---

## 12) 测试 MATLAB 程序的新状态空间方程

1. 复制 `Estimator/M_Estimators/StateSpaceModel1.m` 为 `Estimator/M_Estimators/StateSpaceModel2.m`。
2. 在 `StateSpaceModel2.m` 中将所有 `StateSpaceModel1` 替换为 `StateSpaceModel2`（约 11 处）。
3. 将 `StateSpaceModel2.m` 中使用的估计算法函数由 `Estimator3001` 更改为 `Estimator3002`。
4. 在 `M_Demo.m` 的如下片段之后，追加第二个模型的估计循环（注意写入列偏移从 2:5 改为 6:9）：

```matlab
% —— 既有的 StateSpaceModel1_ 估计流程 ——
% 初始化估计器结构体
StateSpaceModel1_ = struct();
StateSpaceModel1_ = StateSpaceModel1(StateSpaceModel1_);
% 使用 StateSpaceModel1_ 进行估计
for i = 1:DATA_ROWS
    StateSpaceModel1_.CurrentObservation = ReadFileData(i, 2:3)';
    StateSpaceModel1_ = StateSpaceModel1_.EstimatorPort(StateSpaceModel1_);
    WriteFileData(i, 1)   = ReadFileData(i, 1);     % 时间戳
    WriteFileData(i, 2:5) = StateSpaceModel1_.EstimatedState;
end

% —— 追加的 StateSpaceModel2_ 估计流程 ——
StateSpaceModel2_ = struct();
StateSpaceModel2_ = StateSpaceModel2(StateSpaceModel2_);
for i = 1:DATA_ROWS
    StateSpaceModel2_.CurrentObservation = ReadFileData(i, 2:3)';
    StateSpaceModel2_ = StateSpaceModel2_.EstimatorPort(StateSpaceModel2_);
    WriteFileData(i, 1)   = ReadFileData(i, 1);     % 时间戳
    WriteFileData(i, 6:9) = StateSpaceModel2_.EstimatedState;
end
```

完成后运行 `M_Demo.m`，在 `EstimationResult/` 查看新的结果文件。

---

## 13) 更换估计器（C/C++，通过 ID）

* 打开 `Estimator/StateSpaceModel1.c`
* 全局替换 `1001` 为：

  * `1002`（C 语言无迹卡尔曼）
  * `2001`（C++ 线性卡尔曼）
  * `2002`（C++ 无迹卡尔曼）
* 重新构建并运行对应 Demo，比较估计结果。

---

## 14) 修改状态空间模型参数（C/C++ 示例）

在 `Estimator/StateSpaceModel1.c` 中尝试如下修改以观察显著差异：

* 在 `StateSpaceModel1_Initialization / EstimatorPort / Termination` 内，将 `+estimator->Intervel` 与 `+estimator->PredictTime` 改为减号。
* 在系统矩阵 `F` 的定义中，将 `StateSpaceModel1_Interval` 改为 `-StateSpaceModel1_Interval`：

```c
double F[StateSpaceModel1_NX*StateSpaceModel1_NX] = {
    1, StateSpaceModel1_Interval, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, StateSpaceModel1_Interval,
    0, 0, 0, 1
};
```

保存、构建并运行以观察结果变化。

---

## 15) 新建 C 端状态空间模型（StateSpaceModel2 示例）

1. 在 `Estimator/EstimatorPortN.h` 中新增声明：

   ```c
   EXPORT extern EstimatorPortN StateSpaceModel2_;
   EXPORT void StateSpaceModel2_Initialization(struct EstimatorPortN* estimator);
   EXPORT void StateSpaceModel2_EstimatorPort(double* In_Observation, double* Out_State, struct EstimatorPortN* estimator);
   EXPORT void StateSpaceModel2_EstimatorPortTermination(struct EstimatorPortN* estimator);
   ```
2. 复制 `Estimator/StateSpaceModel1.c` 为 `Estimator/StateSpaceModel2.c`，将文件内所有 `StateSpaceModel1_` 替换为 `StateSpaceModel2_`（约 36 处）。
3. 在 `C_Demo.c` 中，参考 `StateSpaceModel1_` 的调用，追加 `StateSpaceModel2_` 的估计流程，并注意将写出列段从 `j+1` 改为 `j+5`（见第 12 节 MATLAB 示例中的写出列偏移思路）。
4. 构建并运行；在 `EstimationResult/` 查看结果。

---

## 16) 常见问题（精简）

* **找不到可执行**：任务输出的可执行位于 `Output/`；请从该目录运行或修正你的运行路径。
* **MATLAB 链接报错 `-lEstimatorPortN: no such file or directory`（Linux）**：确认 `Output/EstimatorPortN.so` 已生成；使用 `['-L' estDir], '-lEstimatorPortN'`；保留 `-Wl,-rpath,'$ORIGIN'`。
* **Python 加载失败（`OSError: cannot open shared object file`）**：从工程根运行，或临时设置 `LD_LIBRARY_PATH`（Linux）/ 添加 `Output/` 至 `PATH`（Windows）。
* **VS Code 提示找不到 `mex.h`**：仅 IntelliSense 问题；为 VS Code 配置 MATLAB `extern/include` 头路径。构建由 MATLAB `mex` 负责，不受此影响。

---

## 17) 许可协议

本工程采用 **MIT License** 开源许可：

```
MIT License

Copyright (c) 2025 Minxing Sun

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 18) Credits

Initial version: **Minxing Sun**
UCAS, Institute of Optics And Electronics, Lab 1, Class of 2020
Email: [401435318@qq.com](mailto:401435318@qq.com)

Project maintenance & cross‑platform build system: collaborative improvements over 2024–2025.

---

> 备注：本文档已尽量去除与个人环境强相关的“任务 JSON 片段”与“命令行细节”，将其视为使用者自定义事项。核心流程均可通过 VS Code 的 **Ctrl + Shift + B** 一键完成，或依据 `CMakeLists.txt` 的目标映射改写。

---
