# ==============================================================================
# Synthesis Automation TCL Script
# Design: foc_controller_top
# ==============================================================================

set DESIGN_NAME "foc_controller_top"

set_app_var target_library "slow_leakage_1v08.db"
set_app_var link_library "* slow_leakage_1v08.db dw_foundation.sldb"

analyze -format verilog {
    ../rtl/transform/clarke_transform.v
    ../rtl/cordic/cordic_rotator.v
    ../rtl/transform/park_transform.v
    ../rtl/pi/pi_controller.v
    ../rtl/transform/inv_park_transform.v
    ../rtl/svpwm/svpwm_generator.v
    ../rtl/estimator/smo_observer.v
    ../rtl/foc_controller_top.v
}

elaborate $DESIGN_NAME
current_design $DESIGN_NAME
link
check_design > reports/check_design.rpt

read_sdc foc_timing.sdc

compile_ultra -gate_clock

report_timing -delay_type max -max_paths 10 > reports/timing_report.txt
report_area -hierarchy > reports/area_report.txt
report_power -hierarchy > reports/power_report.txt
report_qor > reports/qor_report.txt

write -format verilog -hierarchy -output netlist/foc_controller_top_netlist.v
write_sdc netlist/foc_controller_top_synthesized.sdc
exit
