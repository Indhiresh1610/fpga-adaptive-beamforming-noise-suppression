# Vivado Tcl: create the FFT IP used by wiener_cplx_top (instance/module name xfft_0).
# Part: XC7S50-CSGA324-1 (Boolean Board).  Run inside an open project:   source create_xfft_ip.tcl
# If a property name is rejected by your Vivado version, open the IP in the GUI (IP Catalog -> Fast Fourier
# Transform) and set the same options by hand; then CHECK the port widths against wiener_cplx_core.v:
#   s_axis_config_tdata = 8 bit, s_axis_data_tdata = 48 bit, m_axis_data_tdata = 64 bit.
create_ip -name xfft -vendor xilinx.com -library ip -module_name xfft_0
set_property -dict [list \
    CONFIG.transform_length                   {64} \
    CONFIG.target_clock_frequency             {100} \
    CONFIG.implementation_options             {radix_2_burst_io} \
    CONFIG.run_time_configurable_transform_length {false} \
    CONFIG.data_format                        {fixed_point} \
    CONFIG.input_width                        {18} \
    CONFIG.phase_factor_width                 {18} \
    CONFIG.scaling_options                    {unscaled} \
    CONFIG.rounding_modes                     {truncation} \
    CONFIG.output_ordering                    {natural_order} \
    CONFIG.aresetn                            {true} \
] [get_ips xfft_0]
generate_target all [get_ips xfft_0]
