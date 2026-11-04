create_clock -name sys_clk -period 20.000 [get_ports {clk}]
# MIPI byte clock is approximately 74.25 MHz in the supplied 1080p mode.
create_clock -name byte_clk -period 13.468 [get_nets {camera/byte_clk}]
# PLL output frequency follows the camera byte clock; multiply by two.
create_generated_clock -name camera_pixel -source [get_nets {camera/byte_clk}] -multiply_by 2 [get_nets {camera/lvds_pclk}]
create_generated_clock -name tmds_raw -source [get_ports {clk}] -multiply_by 15 [get_nets {camera/raw_serial_clk}]
create_generated_clock -name tmds_serial -source [get_nets {camera/raw_serial_clk}] -divide_by 1 [get_pins {camera/hdmi_clock_gate/CLKOUT}]
create_generated_clock -name hdmi_pixel -source [get_pins {camera/hdmi_clock_gate/CLKOUT}] -divide_by 5 [get_pins {camera/u_clkdiv/CLKOUT}]
# Explicit clock sinks after DHCE: this tool does not reliably propagate the
# gated HCLK through the encrypted DVI wrapper to the serializer clock pins.
create_generated_clock -name oser_serial -source [get_nets {camera/raw_serial_clk}] -divide_by 1 [get_pins {camera/DVI_TX_Top/rgb2dvi_inst/u_OSER10_*/FCLK}]
create_generated_clock -name oser_pixel -source [get_pins {camera/u_clkdiv/CLKOUT}] -divide_by 1 [get_pins {camera/DVI_TX_Top/rgb2dvi_inst/u_OSER10_*/PCLK}]
create_generated_clock -name ddr_memory -source [get_ports {clk}] -multiply_by 4 [get_nets {camera/clk_200M}]
create_generated_clock -name ddr_user -source [get_nets {camera/clk_200M}] -divide_by 4 [get_nets {camera/dma_clk}]

# The official byte/pixel and frame-buffer IPs use asynchronous FIFOs for the
# sensor clock boundaries. They run simultaneously, rather than exclusively.
# GW5A-25 official DDR3 1:4 reference separately groups clkin/clk4x/clk1x:
# https://www.gowinsemi.com/upload/database_doc/2010/document/67cebaccb9842.zip
# (DDR3_MC_PHY_1vs4_5a25k/project/src/ddr3_1v4_hs.sdc).
# The calibrated DDR PHY crosses these domains internally. Frame-buffer FIFO
# ports also cross ddr_user and the independent input/output video clocks.
# Keep sys_clk and hdmi_pixel together so the held yaw bus remains timed.
set_clock_groups -asynchronous -group [get_clocks {byte_clk}] -group [get_clocks {camera_pixel}] -group [get_clocks {ddr_memory}] -group [get_clocks {ddr_user}] -group [get_clocks {sys_clk tmds_raw tmds_serial hdmi_pixel oser_serial oser_pixel}]

# Only the first stages of the yaw toggle synchronizers are false paths.
# The mailbox is held until acknowledge. Its 10 ns bound is less than the
# two 150 MHz pixel periods before the synchronized request can be consumed.
set_false_path -to [get_regs {camera/yaw_osd/req_meta*}]
set_false_path -to [get_regs {camera/yaw_osd/ack_meta*}]
set_max_delay -from [get_regs {camera/yaw_osd/mailbox*}] -to [get_regs {camera/yaw_osd/pixel_text*}] 10.000
set_min_delay -from [get_regs {camera/yaw_osd/mailbox*}] -to [get_regs {camera/yaw_osd/pixel_text*}] 0.000

# Only asynchronous assertion into the three-stage reset synchronizer is
# exempt. Its final Q and all downstream reset recovery paths remain timed.
set_false_path -to [get_pins {camera/hdmi_reset_release/release_pipe*_s0/CLEAR}]
# The DVI common reset is released with its SERIAL AND PIXEL CLOCKS STOPPED.
# hdmi_startup waits 8 sys clocks (160 ns) with DHCE disabled, releases reset,
# then waits another 8 clocks before enabling DHCE. UG306 4.1 specifies this
# dedicated HCLK enable; its official simulation model uses four enable stages.
# Bound physical delivery to 40 ns (rather than ignoring it): at least 120 ns
# remains between reset delivery and the first possible clock. These bounds
# apply only to this sequenced reset and this gate control, not video data.
set_max_delay -through [get_nets {camera/hdmi_ip_reset_n}] -to [get_clocks {oser_serial oser_pixel hdmi_pixel}] 40.000
set_min_delay -through [get_nets {camera/hdmi_ip_reset_n}] -to [get_clocks {oser_serial oser_pixel hdmi_pixel}] 0.000
set_max_delay -from [get_regs {camera/hdmi_startup/clock_disable*}] -to [get_pins {camera/hdmi_clock_gate/CEN}] 40.000
set_min_delay -from [get_regs {camera/hdmi_startup/clock_disable*}] -to [get_pins {camera/hdmi_clock_gate/CEN}] 0.000

# The frame-buffer FIFO releases its read-domain reset on the falling edge.
# Its 150 MHz rising-edge consumers have only a 3.333 ns half cycle. Give
# placement/routing an explicit 3.333 ns recovery-path budget so this
# high-fanout reset receives priority as the video logic grows. This does not
# exempt any endpoint or extend the available recovery time.
set_max_delay -from [get_regs {camera/Video_Frame_Buffer_Top/vfb_ddr3_wrapper_inst/u0_dma_frame_buffer/u_dma_read_ctrl/fifo_read_inst/reset_r_1_s0}] -to [get_clocks {hdmi_pixel}] 3.333
