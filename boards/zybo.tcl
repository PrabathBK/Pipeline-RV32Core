set parent_dir "/Users/tony/arc/pro/dev/svdev/rv32core"
set project_name "rv32zybo"
set top_module "soc"
set part "xc7z010clg400-1"
set device "xc7z010_1"

set project_dir "${parent_dir}/xil/${project_name}"
set project_file "${parent_dir}/xil/${project_name}/${project_name}.xpr"
set program_file "${parent_dir}/xil/${project_name}/${project_name}.runs/impl_1/${top_module}.bit"
set src_dir "${parent_dir}/src"
set tb_dir "${parent_dir}/tb"

open_project $project_file
# update_compile_order -fileset sources_1

open_hw_manager
connect_hw_server -allow_non_jtag
refresh_hw_server
open_hw_target [lindex [get_hw_targets] 0]
current_hw_device [get_hw_devices $device]

reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run synth_1
wait_on_run impl_1

set_property PROBES.FILE {} [get_hw_devices $device]
set_property FULL_PROBES.FILE {} [get_hw_devices $device]
set_property PROGRAM.FILE ${program_file} [get_hw_devices $device]
program_hw_devices [get_hw_devices $device]
refresh_hw_device [lindex [get_hw_devices $device] 0]
# refresh_hw_device -update_hw_probes false [lindex [get_hw_devices xc7z010_1] 0]
close_project
