# init.ps1 - RCIA战队电控组环境初始化 (C/C++混编 + 一键调试版)

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "      🚀 RCIA战队电控组环境初始化 " -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan

# 1. 检查全局环境变量
$ToolRoot = [Environment]::GetEnvironmentVariable("STM32_TOOL_ROOT", "User")
if (-not $ToolRoot) {
    Write-Host "[-] 错误：未发现 STM32_TOOL_ROOT，请先运行 install.ps1。" -ForegroundColor Red
    pause; exit
}

$CleanRoot = $ToolRoot.Replace('\', '/')
Write-Host "[+] 使用全局工具链: $CleanRoot" -ForegroundColor Green

# 2. 定义绝对路径
$GccBin   = "$CleanRoot/gcc/bin"
$CmakeBin = "$CleanRoot/cmake/bin"
$NinjaBin = "$CleanRoot/ninja"
$OcdBin   = "$CleanRoot/openocd/bin"

# 3. 捕获工具链文件
$ToolchainFile = Get-ChildItem -Path $PSScriptRoot -Filter "gcc-arm-none-eabi.cmake" -Recurse | Select-Object -First 1
$ToolchainPath = if ($ToolchainFile) { $ToolchainFile.FullName.Replace('\', '/') } else { "" }

# ========================================================
# [功能]：自动给 CMakeLists.txt 注入 C++ (CXX) 支持
# ========================================================
$CMakeListsPath = Join-Path $PSScriptRoot "CMakeLists.txt"
if (Test-Path $CMakeListsPath) {
    $CMakeContent = Get-Content $CMakeListsPath -Raw
    if ($CMakeContent -match "project\s*\(\s*([^\s]+)\s+C\s+ASM\s*\)") {
        $ProjectName = $matches[1]
        $CMakeContent = $CMakeContent -replace "project\s*\(\s*[^\s]+\s+C\s+ASM\s*\)", "project($ProjectName C CXX ASM)"
        $CMakeContent | Set-Content $CMakeListsPath -Encoding UTF8
        Write-Host "  [√] 核心突破：已自动修改 CMakeLists.txt 开启 C++ 支持" -ForegroundColor Magenta
    }
}

# 4. 排除干扰项
if (Test-Path "CMakePresets.json") { Rename-Item "CMakePresets.json" "CMakePresets.json.bak" -Force -ErrorAction SilentlyContinue }
if (Test-Path "build") { Remove-Item -Recurse -Force "build" -ErrorAction SilentlyContinue }

# 5. 生成 .vscode 目录与 settings.json
$VscodeDir = Join-Path $PSScriptRoot ".vscode"
if (-not (Test-Path $VscodeDir)) { New-Item -ItemType Directory -Path $VscodeDir | Out-Null }

$SettingsObj = [ordered]@{
    "cmake.cmakePath"               = "$CmakeBin/cmake.exe"
    "cmake.generator"               = "Ninja"
    "cmake.environment"             = @{ "PATH" = "$GccBin;${env:PATH}" }
    "cmake.configureSettings"       = @{
        "CMAKE_TOOLCHAIN_FILE"      = $ToolchainPath
        "CMAKE_MAKE_PROGRAM"        = "$NinjaBin/ninja.exe"
        "CMAKE_C_COMPILER"          = "$GccBin/arm-none-eabi-gcc.exe"
        "CMAKE_CXX_COMPILER"        = "$GccBin/arm-none-eabi-g++.exe"
        
    }
    "cmake.useCMakePresets"         = "never"
    "cortex-debug.gdbPath"          = "$GccBin/arm-none-eabi-gdb.exe"
    "cortex-debug.openocdPath"      = "$OcdBin/openocd.exe"
    "cortex-debug.armToolchainPath" = "$GccBin"
    "C_Cpp.intelliSenseEngine"      = "disabled"
    "clangd.arguments"              = @(
        '--compile-commands-dir=${workspaceFolder}/build',
        '--background-index',
        '--header-insertion=never',
        "--query-driver=$GccBin/arm-none-eabi-g*.exe",
        "--query-driver=$GccBin/arm-none-eabi-gcc*.exe"
    )
}
$SettingsObj | ConvertTo-Json -Depth 10 | Set-Content (Join-Path $VscodeDir "settings.json") -Encoding UTF8
Write-Host "  [√] 已生成 settings.json" -ForegroundColor Gray

# ========================================================
# 🌟 [新增功能]：智能侦测芯片 & 生成 launch.json
# ========================================================

# ========================================================
# 🌟 [修复版]：智能侦测芯片 & 生成 launch.json
# ========================================================
Write-Host "`n[?] 请选择你当前使用的调试器:" -ForegroundColor Yellow
Write-Host "  [1] ST-Link (官方推荐 / 战队最常用)"
Write-Host "  [2] DAPLink / CMSIS-DAP (便携开源)"
Write-Host "  [3] J-Link (满血高端)"
$DbgChoice = Read-Host "请输入数字编号并回车 [直接回车默认 ST-Link]"

# 默认兜底配置
$TargetCfg = "stm32f4x"       
$JLinkDev = "STM32F407ZG"     
$IocFile = Get-ChildItem -Path $PSScriptRoot -Filter "*.ioc" | Select-Object -First 1

if ($IocFile) {
    # 【关键修复】：加入 -Raw 参数，将文件作为单个字符串读取
    $IocContent = Get-Content $IocFile.FullName -Raw 
    
    # 侦测 OpenOCD 目标家族
    if ($IocContent -match "Mcu\.Family=(STM32\w+)") {
        $TargetCfg = $Matches[1].ToLower() + "x"
        Write-Host "  [+] 自动侦测到 OpenOCD 目标: target/$TargetCfg.cfg" -ForegroundColor Green
    }
    
    # 侦测 J-Link 具体型号
    if ($IocContent -match "Mcu\.UserName=([A-Z0-9xX]+)") {
        $JLinkDev = $Matches[1]
        Write-Host "  [+] 自动侦测到 J-Link 目标: $JLinkDev" -ForegroundColor Green
    }
} else {
    Write-Host "  [!] 未找到 .ioc 文件，将使用默认配置 (STM32F4)。" -ForegroundColor DarkYellow
}


# 构建 launch.json 结构
$LaunchConfig = [ordered]@{
    "version" = "0.2.0"
    "configurations" = @()
}

$BaseDbgConfig = [ordered]@{
    "name" = "🔥 Run / Debug"
    "cwd" = '${workspaceFolder}'
    "executable" = '${command:cmake.launchTargetPath}'
    "request" = "launch"
    "type" = "cortex-debug"
    "runToEntryPoint" = "main"
    "showDevDebugOutput" = "none"
}

if ($DbgChoice -eq "2") {
    $BaseDbgConfig["servertype"] = "openocd"
    $BaseDbgConfig["configFiles"] = @("interface/cmsis-dap.cfg", "target/$TargetCfg.cfg")
} elseif ($DbgChoice -eq "3") {
    $BaseDbgConfig["servertype"] = "jlink"
    $BaseDbgConfig["device"] = $JLinkDev
    $BaseDbgConfig["interface"] = "swd"
} else {
    # 默认 ST-Link
    $BaseDbgConfig["servertype"] = "openocd"
    $BaseDbgConfig["configFiles"] = @("interface/stlink.cfg", "target/$TargetCfg.cfg")
}

$LaunchConfig["configurations"] += $BaseDbgConfig
$LaunchConfig | ConvertTo-Json -Depth 10 | Set-Content (Join-Path $VscodeDir "launch.json") -Encoding UTF8
Write-Host "  [√] 已生成 launch.json 调试配置" -ForegroundColor Gray
# ========================================================

# 7. 生成扩展推荐配置
$ExtensionsFile = Join-Path $VscodeDir "extensions.json"
$ExtensionsConfig = [ordered]@{
    "recommendations" = @("marus25.cortex-debug", "ms-vscode.cmake-tools", "llvm-vs-code-extensions.vscode-clangd", "ms-vscode.cpptools")
}
$ExtensionsConfig | ConvertTo-Json -Depth 5 | Set-Content $ExtensionsFile -Encoding UTF8
Write-Host "  [√] 已生成 VS Code 推荐扩展列表" -ForegroundColor Gray

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  ✅ 项目环境配置成功！点击 Trae 左侧的【运行和调试】即可起飞！" -ForegroundColor Green
Write-Host "================================================" -ForegroundColor Cyan
pause