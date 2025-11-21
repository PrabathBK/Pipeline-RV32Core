# Set up directory and file paths
set parent_dir "/Users/tony/arc/pro/dev/svdev/rv32core"
set top_module "soc"
set part "xc7z010clg400-1"
set device "xc7z010_1"
set src_dir "${parent_dir}/src"
set boards_dir "${parent_dir}/boards"
set program_file "${parent_dir}/out/${top_module}.bit"
set checkpoint_dir "${parent_dir}/out/checkpoints"
set num_jobs 8

# Set number of parallel jobs for synthesis
set_param general.maxThreads $num_jobs

# Create output directories
file mkdir "${parent_dir}/out"
file mkdir $checkpoint_dir

# Read source files with error checking
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
    global top_module part program_file checkpoint_dir

    # Check if there's a previous checkpoint
    set prev_checkpoint "${checkpoint_dir}/post_route.dcp"
    set incremental_flag {}
    if {[file exists $prev_checkpoint]} {
        set incremental_flag "-incremental $prev_checkpoint"
    }

    # Synthesis with optimization flags
    if {[catch {synth_design -top $top_module -part $part \
        -directive RuntimeOptimized \
        -flatten_hierarchy rebuilt \
        -gated_clock_conversion on \
        -resource_sharing on } err]} {
        puts "Synthesis failed: $err"
        return -1
    }
    write_checkpoint -force "${checkpoint_dir}/post_synth.dcp"

    # Optimize with timing focus
    if {[catch {opt_design -directive ExploreWithRemap} err]} {
        puts "Optimization failed: $err"
        return -1
    }
    write_checkpoint -force "${checkpoint_dir}/post_opt.dcp"

    # Place with timing focus
    if {[catch {place_design -directive RuntimeOptimized} err]} {
        puts "Placement failed: $err"
        return -1
    }
    write_checkpoint -force "${checkpoint_dir}/post_place.dcp"

    # Physical optimization focused on timing
    if {[catch {phys_opt_design -directive AggressiveExplore} err]} {
        puts "Physical optimization failed: $err"
        return -1
    }
    write_checkpoint -force "${checkpoint_dir}/post_physopt.dcp"

    # Route with timing focus
    if {[catch {route_design -directive RuntimeOptimized} err]} {
        puts "Routing failed: $err"
        return -1
    }
    write_checkpoint -force "${checkpoint_dir}/post_route.dcp"

    # Generate reports
    report_timing_summary -file "${parent_dir}/out/timing.rpt"
    report_utilization -file "${parent_dir}/out/utilization.rpt"
    report_power -file "${parent_dir}/out/power.rpt"

    # Write bitstream with compression for faster programming
    write_bitstream -force -bin_file -compress $program_file
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
