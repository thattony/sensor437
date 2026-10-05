$env:Path = "C:\Xilinx\Vivado\2022.2\bin;$env:Path"
$env:OK_API_PATH = "C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64"

$bash = "C:\Program Files\Git\bin\bash.exe"
$python = ".\.venv310\Scripts\python.exe"
$dll = "C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64"

Write-Host "`n===== 1. BUILD STATUS ====="
& $bash ./build.sh --status

Write-Host "`n===== BUILD LOG ====="
Get-Content .\build\build.log -Tail 1

Read-Host "Press Enter for smoke test"

Write-Host "`n===== 2. SMOKE TEST ====="
& $python -c "import os,sys,runpy; os.add_dll_directory(r'$dll'); sys.path.insert(0,r'.\python'); runpy.run_path(r'.\python\smoke_test.py', run_name='__main__')"

Read-Host "Press Enter for I2C device scan"

Write-Host "`n===== 3. I2C DEVICE SCAN ====="
& $python -c "import os,sys,runpy; os.add_dll_directory(r'$dll'); sys.path.insert(0,r'.\python'); runpy.run_path(r'.\python\i2c_first_frame.py', run_name='__main__')"

Read-Host "Press Enter for simulation"

# Write-Host "`n===== 4. SIMULATION ====="
# & $bash ./build.sh --sim

Write-Host "`n===== SIM LOG ====="
Get-Content .\sim\sim.log -Tail 160

Read-Host "Press Enter for ADT7420 ID REGISTER"

Write-Host "`n===== M2.2 HARDWARE: ADT7420 ID REGISTER ====="
& $python .\python\lab6_python.py --addr 0x48 --reg 0x0B

Read-Host "`nPress Enter for absent-device test"

Write-Host "`n===== M2.3 HARDWARE: ABSENT DEVICE 0x49 ====="
& $python .\python\lab6_python.py --addr 0x49 --reg 0x0B