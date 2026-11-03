// top_touch_synth.sdc —— 时序约束：核心板 50 MHz 时钟（全部逻辑都在这个时钟域）
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {i_clk_50m}]
