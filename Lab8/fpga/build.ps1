# build.ps1 - Lab 8: build fpga\bitfile\lab8_motor.bit from ..\lab8_motor.v (Vivado 2022.2, ~10 min)
#   powershell -ExecutionPolicy Bypass -File build.ps1          build the bit file
#   powershell -ExecutionPolicy Bypass -File build.ps1 -Sim     run sim\tb_lab8_core.v (log: sim\sim.log)
param([switch]$Sim)
$ErrorActionPreference = "Continue"
$env:Path = "C:\Xilinx\Vivado\2022.2\bin;$env:Path"
Set-Location $PSScriptRoot
if ($Sim) {
    Set-Location sim
    xvlog -work work ..\..\lab8_motor.v i2c_slave_model.v tb_lab8_core.v | Out-Null
    xelab -debug typical -top tb_lab8_core -snapshot tb_snap | Out-Null
    xsim tb_snap -R | Out-File -Encoding utf8 sim_full.log
    Select-String -Path sim_full.log -Pattern "PASS|FAIL|ALL CHECKS" | ForEach-Object { $_.Line } | Tee-Object sim.log
    exit 0
}
New-Item -ItemType Directory -Force build | Out-Null
vivado -mode batch -source vivado/build.tcl -log build/vivado.log -journal build/vivado.jou *>&1 | Out-File -Encoding utf8 build/build.log
"EXIT_CODE=$LASTEXITCODE" | Out-File -Append -Encoding utf8 build/build.log
Select-String -Path build/build.log -Pattern "BUILD COMPLETE|^ERROR|Post-route timing|IP BOARD =" | ForEach-Object { $_.Line }
