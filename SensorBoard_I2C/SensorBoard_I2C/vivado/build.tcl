# ============================================================================
# build.tcl - Batch build of bitfile/SensorBoard_Top.bit for the OpalKelly
#             XEM7310-A75 (Artix-7 xc7a75tfgg484-1). Verified with Vivado 2024.1.
#
# Run:  ./build.sh            (finds Vivado natively or in the Docker container)
#  or:  vivado -mode batch -source vivado/build.tcl -log build/vivado.log -journal build/vivado.jou
#
# Flow: project -> every hdl/*.v + constraints/*.xdc -> FrontPanel Subsystem IP
#       -> synthesis -> implementation -> bitstream -> bitfile/SensorBoard_Top.bit
# ============================================================================

set script_dir   [file dirname [file normalize [info script]]]
set proj_root    [file normalize "$script_dir/.."]
cd $proj_root

set project_name "SensorBoard"
set top_name     "SensorBoard_Top"
set fpga_part    "xc7a75tfgg484-1"
set fp_board     "XEM7310-A75"
set build_dir    "$proj_root/build"
set proj_dir     "$build_dir/vivado"
set bit_dir      "$proj_root/bitfile"
set jobs         4

puts "======================================"
puts "Project root : $proj_root"
puts "Build dir    : $build_dir"
puts "Part         : $fpga_part  (FrontPanel board: $fp_board)"
puts "Vivado       : [version -short]"
puts "======================================"

file mkdir $build_dir
file mkdir $bit_dir
if {[file exists $proj_dir]} { file delete -force $proj_dir }

# ----------------------------------------------------------------------------
# Project + sources: every Verilog file in hdl/ and every XDC in constraints/.
# Add a new module = drop a .v file into hdl/. Simulation-only files live in
# sim/ and are added to the simulation fileset (not synthesised).
# ----------------------------------------------------------------------------
create_project $project_name $proj_dir -part $fpga_part -force

set hdl_files [lsort [glob -nocomplain "$proj_root/hdl/*.v" "$proj_root/hdl/*.sv"]]
if {[llength $hdl_files] == 0} { puts "ERROR: no HDL files in $proj_root/hdl"; exit 1 }
puts "HDL files: $hdl_files"
add_files $hdl_files

set xdc_files [lsort [glob -nocomplain "$proj_root/constraints/*.xdc"]]
puts "XDC files: $xdc_files"
add_files -fileset constrs_1 $xdc_files

set sim_files [lsort [glob -nocomplain "$proj_root/sim/*.v" "$proj_root/sim/*.sv"]]
if {[llength $sim_files] > 0} {
    add_files -fileset sim_1 $sim_files
    set_property top tb_i2c_transmit [get_filesets sim_1]
}

set_property top $top_name [current_fileset]
update_compile_order -fileset sources_1

# ----------------------------------------------------------------------------
# FrontPanel Subsystem IP (OpalKelly). CONFIG.BOARD MUST be XEM7310-A75:
# left unset, Vivado picks XEM7310MT-A75 and the host interface never
# enumerates (bitstream loads, but IsFrontPanelEnabled() is False).
#
# Endpoint map - keep in sync with hdl/SensorBoard_Top.v and python/sensor_board.py
#   WireIn  0x00 ctrl, 0x01..0x03 param1..3
#   WireOut 0x20..0x22 result0..2, 0x23 status, 0x3F design ID
#   TriggerIn 0x40 (start/reset), TriggerOut 0x60 (done), PipeOut 0xA0
# ----------------------------------------------------------------------------
set fp_ip "$proj_root/opalkelly/ip_repos/FrontPanel-Vivado-IP-Dist-v1.0.6/FrontPanel-Subsystem-v1.0.6"
if {![file exists $fp_ip]} {
    puts "ERROR: FrontPanel Subsystem IP repository not found at $fp_ip"
    exit 1
}
puts "FrontPanel IP repo: $fp_ip"
set_property ip_repo_paths [list $fp_ip] [current_project]
update_ip_catalog

puts "Creating frontpanel_0 ..."
create_ip -name frontpanel -vendor opalkelly.com -library ip -version 1.0 -module_name frontpanel_0
set_property -dict [list \
    CONFIG.BOARD     $fp_board \
    CONFIG.WI.COUNT  {4} \
    CONFIG.WI.ADDR_0 {0x00} \
    CONFIG.WI.ADDR_1 {0x01} \
    CONFIG.WI.ADDR_2 {0x02} \
    CONFIG.WI.ADDR_3 {0x03} \
    CONFIG.WO.COUNT  {5} \
    CONFIG.WO.ADDR_0 {0x20} \
    CONFIG.WO.ADDR_1 {0x21} \
    CONFIG.WO.ADDR_2 {0x22} \
    CONFIG.WO.ADDR_3 {0x23} \
    CONFIG.WO.ADDR_4 {0x3F} \
    CONFIG.TI.COUNT  {1} \
    CONFIG.TI.ADDR_0 {0x40} \
    CONFIG.TO.COUNT  {1} \
    CONFIG.TO.ADDR_0 {0x60} \
    CONFIG.PO.COUNT  {1} \
    CONFIG.PO.ADDR_0 {0xA0} \
] [get_ips frontpanel_0]
set ip_board [get_property CONFIG.BOARD [get_ips frontpanel_0]]
puts "IP BOARD = $ip_board"
if {$ip_board ne $fp_board} {
    puts "ERROR: frontpanel_0 BOARD is '$ip_board', expected '$fp_board'"
    exit 1
}
generate_target all [get_ips frontpanel_0]

# ----------------------------------------------------------------------------
# Synthesis
# ----------------------------------------------------------------------------
puts "======================================"
puts "Synthesis..."
puts "======================================"
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
set synth_status [get_property STATUS   [get_runs synth_1]]
set synth_prog   [get_property PROGRESS [get_runs synth_1]]
puts "synth_1: $synth_status ($synth_prog)"
if {$synth_prog != "100%"} {
    puts "ERROR: synthesis did not complete. See $proj_dir/${project_name}.runs/synth_1/runme.log"
    exit 1
}

open_run synth_1
report_utilization    -file $build_dir/post_synth_utilization.rpt
report_utilization    -hierarchical -file $build_dir/post_synth_utilization_hier.rpt
report_timing_summary -file $build_dir/post_synth_timing.rpt

# Sanity: the I2C pads must be bidirectional (input + tristate output buffer).
# (A line the FSM never pulls low has no buffer at all - that is what the
#  placeholder FSM looks like. A real I2C master shows IBUF + OBUFT or IOBUF.)
foreach p {I2C_SCL_0 I2C_SDA_0 I2C_SCL_1 I2C_SDA_1} {
    set cells [get_cells -quiet -of_objects [get_nets -quiet -of_objects [get_ports -quiet $p]]]
    if {[llength $cells] == 0} {
        puts "I2C PAD $p: no buffer (line unused or constant in this design)"
    } else {
        puts "I2C PAD $p: [lsort -unique [get_property REF_NAME $cells]]"
    }
}
close_design

# ----------------------------------------------------------------------------
# Implementation + bitstream
# ----------------------------------------------------------------------------
puts "======================================"
puts "Implementation + bitstream..."
puts "======================================"
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
set impl_status [get_property STATUS   [get_runs impl_1]]
set impl_prog   [get_property PROGRESS [get_runs impl_1]]
puts "impl_1: $impl_status ($impl_prog)"

set bit_src "$proj_dir/${project_name}.runs/impl_1/${top_name}.bit"
if {![file exists $bit_src]} {
    puts "ERROR: bitstream not produced ($impl_status). See $proj_dir/${project_name}.runs/impl_1/runme.log"
    exit 1
}

open_run impl_1
report_timing_summary -file $build_dir/post_route_timing.rpt
report_utilization    -file $build_dir/post_route_utilization.rpt
report_utilization    -hierarchical -file $build_dir/post_route_utilization_hier.rpt
report_drc            -file $build_dir/post_route_drc.rpt
report_io             -file $build_dir/post_route_io.rpt
set wns [get_property -quiet STATS.WNS [get_runs impl_1]]
set whs [get_property -quiet STATS.WHS [get_runs impl_1]]
puts "Post-route timing: WNS = $wns ns, WHS = $whs ns"
if {($wns ne "" && $wns < 0) || ($whs ne "" && $whs < 0)} {
    puts "WARNING: timing violation - bitstream was still written; inspect $build_dir/post_route_timing.rpt"
}
close_design

file copy -force $bit_src "$build_dir/${top_name}.bit"
file copy -force $bit_src "$bit_dir/${top_name}.bit"

puts "======================================"
puts "BUILD COMPLETE"
puts "Bitstream : $bit_dir/${top_name}.bit"
puts "Reports   : $build_dir/  (utilization of I2C_Transmit alone: post_synth_utilization_hier.rpt, row u_i2c)"
puts "IP board  : $ip_board"
puts "======================================"
close_project
