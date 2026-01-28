**English** | [中文](README.zh-CN.md)
# EstimatorFrame — Estimation Interfaces & Multi‑Language Calling

> A unified project for **Windows / Linux** that compiles the estimator core (C/C++) into a shared library and demonstrates how to call it from **C / C++ / Python / MATLAB**. The documentation follows a clear directory convention and one‑click builds via **VS Code Tasks**.

---

## 1) Directory & Artifact Conventions

```
EstimatorFrame/           # Project root
├─ Estimator/             # Shared libs source file and headers
│  ├─ EstimatorPortN.h
│  ├─ EstimatorPortN_MatlabDLL.cpp             (MATLAB <-> DLL bridge, Windows)
│  └─ EstimatorPortN_MatlabSO.cpp              (MATLAB <-> SO  bridge, Linux)
├─ Output/                # Executables and Shared libs (*.dll / *.lib / *.so / *.mexa64)
│  ├─ EstimatorPortN.dll / EstimatorPortN.lib   (Windows)
│  ├─ EstimatorPortN.so                         (Linux)
├─ Temp/                  # Intermediates (build dir, temp files, etc.)
├─ ObservationData/       # Observation data (default demo)
├─ EstimationResult/      # Estimation results (*.txt with timestamp)
└─ *.c / *.cpp / *.py / *.m
```

> **Run location**: always start your terminal from the project root (or run tasks from VS Code workspace root).

---

## 2) Prerequisites

### Windows

* Microsoft Visual Studio **Build Tools 2022** (MSVC, NMake)
* **CMake ≥ 3.16**
* **Python 3.x** (optional, for Python demo)
* **MATLAB R2020a+** (optional, for MATLAB demo)
* Use the VS Build Tools environment script (`vcvars64.bat`) to initialize the toolchain (pre‑wired in the tasks).

### Linux (verified on Ubuntu 20.04+)

* **gcc/g++**, **make**
* **CMake ≥ 3.16**
* **Python 3.x** (optional)
* **MATLAB R2020a+** (optional)

---

## 3) Quick Start (Recommended: VS Code Tasks)

Press **Ctrl + Shift + B**, then choose among:

* **Build EstimatorPortN (Windows/Linux)**: builds shared lib → `Output/EstimatorPortN.dll` (+`EstimatorPortN.lib`) or `Output/EstimatorPortN.so`
* **Build C_Demo / Cpp_Demo**: directly links estimator sources into executable → `Output/C_Demo` / `Output/Cpp_Demo`
* **Build C_DLL_Demo / Cpp_DLL_Demo (Windows)**: runtime‑loads `Output/EstimatorPortN.dll` → `Output/*.exe`
* **Build C_SO_Demo / Cpp_SO_Demo (Linux)**: runtime‑loads `Output/EstimatorPortN.so` → `Output/*`

> **Tip**: tasks already set up outputs and dependencies—no need to type commands manually. If you prefer CLI, consult `CMakeLists.txt` for targets and output paths (omitted here for brevity).

---

## 4) Test Data

* Default: `ObservationData/DoubleReflectorTrace/Trace1000.txt`
* You may replace with your own observation data (keep format/dimensions consistent).

---

## 5) C Program (Direct Link)

1. **Build**: pick `Build C_Demo (Windows/Linux)`
2. **Run**:

   * Windows: `Output\C_Demo.exe`
   * Linux: `./Output/C_Demo`
3. **Expected output** (example):

   ```
   Tested Estimator v0.00 is initialized
   Unscented Kalman estimator is initialized
   EstimatorPort terminated.
   StateSpaceModel1_ finished...
   Estimation data has been written to EstimationResult/EstimationResult_YYYYMMDDhhmmss.txt
   Program terminated...
   ```
4. Result files go to `EstimationResult/`.

---

## 6) C++ Program (Direct Link)

1. **Build**: pick `Build Cpp_Demo (Windows/Linux)`
2. **Run**:

   * Windows: `Output\Cpp_Demo.exe`
   * Linux: `./Output/Cpp_Demo`
3. Check results under `EstimationResult/`.

---

## 7) Build Shared Library & Runtime‑Load Examples

### 7.1 Build the Shared Library

* Windows: `Build EstimatorPortN (Windows, DLL -> Output/)` → `Output/EstimatorPortN.dll` and `Output/EstimatorPortN.lib`
* Linux: `Build EstimatorPortN (Linux, SO -> Output/)` → `Output/EstimatorPortN.so`

### 7.2 Runtime Loading (Windows: DLL, Linux: SO)

* **Windows**:

  * Choose `Build C_DLL_Demo` / `Build Cpp_DLL_Demo`
  * Run `Output\C_DLL_Demo.exe` / `Output\Cpp_DLL_Demo.exe`
  * **Requirement**: at runtime, ensure `Output/` is on `PATH`, or run from project root so `Output\EstimatorPortN.dll` is discoverable by relative path.

* **Linux**:

  * Choose `Build C_SO_Demo` / `Build Cpp_SO_Demo`
  * Run `./Output/C_SO_Demo` / `./Output/Cpp_SO_Demo`
  * If your system lacks proper rpath, temporarily: `export LD_LIBRARY_PATH="$PWD/Output:$LD_LIBRARY_PATH"`

---

## 8) Python Calling the Shared Library

### 8.1 Windows: `Python_DLL_Demo.py`

* From project root: `python Python_DLL_Demo.py`
* Requires `Output/EstimatorPortN.dll` to be loadable (run from root or add `Output/` to `PATH`).

### 8.2 Linux: `Python_SO_Demo.py`

* From project root: `python3 Python_SO_Demo.py`
* Requires `Output/EstimatorPortN.so` to be loadable (run from root or set `LD_LIBRARY_PATH` as above).

> Python can also call `Output/*` executables via `subprocess.run([...])`, but the shared‑library approach is more efficient and consistent across languages.

---

## 9) MATLAB (Windows, DLL)

We recommend embedding a **one‑time build** inside your M‑file: first run will generate the MEX; later runs won’t need to rebuild.

```matlab
% ===== Windows + DLL: One‑time MEX build (if not already present) =====
projRoot = fileparts(mfilename('fullpath'));
estDir   = fullfile(projRoot, 'Estimator');
outDir   = fullfile(projRoot, 'Output');
srcCpp   = fullfile(estDir, 'EstimatorPortN_MatlabDLL.cpp');
outBase  = fullfile(outDir, 'EstimatorPortN_MatlabDLL');  % without extension
outMex   = [outBase '.' mexext];                          % expected .mexw64

if ~exist(outMex, 'file')
    % First time only: you may run once:  mex -setup C++
    mex('-v', ['-I' estDir], srcCpp, fullfile(outDir, 'EstimatorPortN.lib'), ...
        '-output', outBase);
end

% Then your M code can call: EstimatorPortN_MatlabDLL('initialize'|'estimate'|'terminate')
```

> **Note**: on Windows, we recommend passing the full path to `.lib` (see `fullfile(estDir,'EstimatorPortN.lib')`). The MEX will be placed in `Output/`, alongside `EstimatorPortN.dll`, for easy runtime loading.

---

## 10) MATLAB (Linux, SO)

Likewise, embed a one‑time auto‑build and set rpath so the MEX can find the `.so` next to itself:

```matlab
% ===== Linux + SO: One‑time MEX build (if not already present) =====
projRoot = fileparts(mfilename('fullpath'));
estDir   = fullfile(projRoot, 'Estimator');
outDir   = fullfile(projRoot, 'Output');
srcCpp   = fullfile(estDir, 'EstimatorPortN_MatlabSO.cpp');
outBase  = fullfile(outDir, 'EstimatorPortN_MatlabSO');   % without extension
outMex   = [outBase '.' mexext];                          % expected .mexa64

if ~exist(outMex, 'file')
    % First time only: you may run once:  mex -setup C++
    mex('-v', ['-I' estDir], ...                      % headers
        srcCpp, ...                                   % source
        ['-L' outDir], '-lEstimatorPortN', ...        % link EstimatorPortN.so
        "LDFLAGS=$LDFLAGS -Wl,-rpath,'$ORIGIN'", ... % find .so next to MEX at runtime
        '-output', outBase);
end

% Then your M code can call: EstimatorPortN_MatlabSO('initialize'|'estimate'|'terminate')
```

> **If you revert CMake to default prefix** (producing `libEstimatorPortN.so`), you can still link with `-lEstimatorPortN`; the linker strips `lib` and `.so` automatically.

---

## 11) Test MATLAB Program (Pure .m)

1. Open and run `M_Demo.m`. It invokes `Estimator/M_Estimators/StateSpaceModel1.m`, which internally uses `Estimator/M_Estimators/Estimator3001.m`.
2. After completion, check `EstimationResult/` for `EstimationResult_YYYYMMDDhhmmss.txt`.

---

## 12) Test MATLAB Program with a New State‑Space Model

1. Copy `Estimator/M_Estimators/StateSpaceModel1.m` to `Estimator/M_Estimators/StateSpaceModel2.m`.
2. In `StateSpaceModel2.m`, replace all `StateSpaceModel1` with `StateSpaceModel2` (≈11 occurrences).
3. Change the estimator function used in `StateSpaceModel2.m` from `Estimator3001` to `Estimator3002`.
4. In `M_Demo.m`, append the second model’s loop **after** the snippet below (note the write column offset from `2:5` to `6:9`):

```matlab
% —— Existing StateSpaceModel1_ estimation ——
% Initialize estimator struct
StateSpaceModel1_ = struct();
StateSpaceModel1_ = StateSpaceModel1(StateSpaceModel1_);
% Use StateSpaceModel1_ for estimation
for i = 1:DATA_ROWS
    StateSpaceModel1_.CurrentObservation = ReadFileData(i, 2:3)';
    StateSpaceModel1_ = StateSpaceModel1_.EstimatorPort(StateSpaceModel1_);
    WriteFileData(i, 1)   = ReadFileData(i, 1);     % timestamp
    WriteFileData(i, 2:5) = StateSpaceModel1_.EstimatedState;
end

% —— Added StateSpaceModel2_ estimation ——
StateSpaceModel2_ = struct();
StateSpaceModel2_ = StateSpaceModel2(StateSpaceModel2_);
for i = 1:DATA_ROWS
    StateSpaceModel2_.CurrentObservation = ReadFileData(i, 2:3)';
    StateSpaceModel2_ = StateSpaceModel2_.EstimatorPort(StateSpaceModel2_);
    WriteFileData(i, 1)   = ReadFileData(i, 1);     % timestamp
    WriteFileData(i, 6:9) = StateSpaceModel2_.EstimatedState;
end
```

Run `M_Demo.m` and check `EstimationResult/` for the new result file.

---

## 13) Switch Estimators by ID (C/C++)

* Open `Estimator/StateSpaceModel1.c`
* Globally replace `1001` with:

  * `1002` (C UKF)
  * `2001` (C++ linear KF)
  * `2002` (C++ UKF)
* Rebuild and run the demos; compare outputs.

---

## 14) Modify State‑Space Parameters (C/C++ Example)

In `Estimator/StateSpaceModel1.c`, try the following to observe noticeable differences:

* In `StateSpaceModel1_Initialization / EstimatorPort / Termination`, change `+estimator->Intervel` and `+estimator->PredictTime` to minus signs.
* In system matrix `F`, change `StateSpaceModel1_Interval` to `-StateSpaceModel1_Interval`:

```c
double F[StateSpaceModel1_NX*StateSpaceModel1_NX] = {
    1, StateSpaceModel1_Interval, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, StateSpaceModel1_Interval,
    0, 0, 0, 1
};
```

Save, build, and run to compare results.

---

## 15) Create a New C‑Side State‑Space Model (StateSpaceModel2 Example)

1. Add the following declarations to `Estimator/EstimatorPortN.h`:

   ```c
   EXPORT extern EstimatorPortN StateSpaceModel2_;
   EXPORT void StateSpaceModel2_Initialization(struct EstimatorPortN* estimator);
   EXPORT void StateSpaceModel2_EstimatorPort(double* In_Observation, double* Out_State, struct EstimatorPortN* estimator);
   EXPORT void StateSpaceModel2_EstimatorPortTermination(struct EstimatorPortN* estimator);
   ```
2. Copy `Estimator/StateSpaceModel1.c` to `Estimator/StateSpaceModel2.c`, and replace all `StateSpaceModel1_` with `StateSpaceModel2_` (≈36 occurrences).
3. In `C_Demo.c`, mirror the `StateSpaceModel1_` flow for `StateSpaceModel2_`, and **change write indices** from `j+1` to `j+5` (see the write‑column offset logic in the MATLAB example in §12).
4. Build and run; check results under `EstimationResult/`.

---

## 16) Common Issues (Concise)

* **Executable not found**: outputs go to `Output/`. Run from there or fix your run path.
* **MATLAB link error `-lEstimatorPortN: no such file or directory` (Linux)**: ensure `Output/EstimatorPortN.so` exists; use `['-L' estDir], '-lEstimatorPortN'`; keep `-Wl,-rpath,'$ORIGIN'`.
* **Python load failure (`OSError: cannot open shared object file`)**: run from project root, or set `LD_LIBRARY_PATH` (Linux) / add `Output/` to `PATH` (Windows).
* **VS Code shows missing `mex.h`**: IntelliSense only—configure MATLAB `extern/include` in VS Code. MATLAB’s `mex` build is not affected.

---

## 17) License

This project is released under the **MIT License**:

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

> Note: this guide deliberately avoids environment‑specific “task JSON snippets” and long command listings. Treat those as user‑specific choices. All core flows can be triggered via VS Code **Ctrl + Shift + B**, or adapted from the targets defined in `CMakeLists.txt`.
