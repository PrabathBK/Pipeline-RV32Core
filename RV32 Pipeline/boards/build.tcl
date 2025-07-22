# Set up directory and file paths
set parent_dir "/Users/tony/arc/pro/dev/svdev/rv32core"
set top_module "soc"
set part "xc7z010clg400-1"
set device "xc7z010_1"
set src_dir "${parent_dir}/src"
set boards_dir "${parent_dir}/boards"
set program_file "${parent_dir}/out/${top_module}.bit"
set num_jobs 8

# Set number of parallel jobs for synthesis
set_param general.maxThreads $num_jobs

# Create output directory
file mkdir "${parent_dir}/out"

# clear previous run
# write_hw_platform -force -fixed -include_bit "${parent_dir}/out/${top_module}.xsa"

# read source files with error checking
proc read_source_files {} {
    global src_dir
    foreach file [glob -nocomplain ${src_dir}/*.sv] {
        if {[catch {read_verilog -sv $file} err]} {
            puts "Error reading $file: $err"
            return -1
        }
    }
    foreach file [glob -nocomplain ${src_dir}/*.v] {
        if {[catch {read_verilog $file} err]} {
            puts "Error reading $file: $err"
            return -1
        }
    }
    foreach file [glob -nocomplain ${src_dir}/*.xdc] {
        if {[catch {read_xdc $file} err]} {
            puts "Error reading $file: $err"
            return -1
        }
    }
    return 0
}

# Implementation with timing and error checks
proc implement_design {} {
    global top_module part program_file

    if {[catch {synth_design -top $top_module -part $part} err]} {
        puts "Synthesis failed: $err"
        return -1
    }

    if {[catch {opt_design} err]} {
        puts "Optimization failed: $err"
        return -1
    }

    if {[catch {place_design} err]} {
        puts "Placement failed: $err"
        return -1
    }

    if {[catch {phys_opt_design} err]} {
        puts "Physical optimization failed: $err"
        return -1
    }

    if {[catch {route_design} err]} {
        puts "Routing failed: $err"
        return -1
    }

    write_bitstream -force $program_file
    return 0
}

# Program FPGA
proc program_fpga {} {
    global program_file device

    open_hw_manager
    connect_hw_server -allow_non_jtag
    open_hw_target
    current_hw_device [lindex [get_hw_devices $device] 0]
    set_property PROGRAM.FILE ${program_file} [current_hw_device]
    program_hw_devices [current_hw_device]
    close_hw_target
    close_hw_manager
}

# Main execution
if {[read_source_files] == 0} {
    if {[implement_design] == 0} {
        program_fpga
    }
}
