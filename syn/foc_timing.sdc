# ==============================================================================
# Synopsys Design Constraints (SDC)
# Target: Sensorless FOC Motor Controller ASIC Core
# Process Technology: SkyWater 130nm HD Standard Cell Library (1.80V, 25C)
# Operating Conditions: Fast-Fast (0.99V, 0C) / Slow-Slow (0.81V, 125C)
# Target Frequency: 100 MHz (Clock Period = 10.0 ns)
# ==============================================================================

create_clock -name clk -period 10.000 [get_ports clk]

# Clock Uncertainty, Jitter & Duty Cycle Distortion
set_clock_uncertainty -setup 0.200 [get_clocks clk]
set_clock_uncertainty -hold 0.050 [get_clocks clk]
set_clock_transition 0.120 [get_clocks clk]

# Input / Output Port Constraints (ADC Current Feedback & Inverter PWM Gate Drives)
set_input_delay -clock clk -max 2.000 [get_ports {adc_ia adc_ib sample_trigger ext_theta id_ref iq_ref foc_enable sensorless_mode}]
set_input_delay -clock clk -min 0.400 [get_ports {adc_ia adc_ib sample_trigger ext_theta id_ref iq_ref foc_enable sensorless_mode}]

set_output_delay -clock clk -max 2.000 [get_ports {pwm_a pwm_an pwm_b pwm_bn pwm_c pwm_cn svpwm_sector}]
set_output_delay -clock clk -min 0.300 [get_ports {pwm_a pwm_an pwm_b pwm_bn pwm_c pwm_cn svpwm_sector}]

# Drive Strength & Capacitive Load Constraints
set_driving_cell -lib_cell INVX4 [all_inputs]
set_load -pin_load 0.040 [all_outputs]

# Timing Exceptions
set_false_path -from [get_ports rst_n]
