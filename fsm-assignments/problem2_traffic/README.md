# Traffic Light FSM (NS/EW control)

## How 1 Hz Tick Was Generated
- In real hardware, a clock divider would generate a 1 Hz pulse from the main `clk`.
- For simulation, `tick` is directly toggled every simulated second.

## Simulation Instructions

### Using Icarus Verilog + GTKWave
```bash
iverilog -o sim.out tb_traffic_light.v traffic_light.v
vvp sim.out
gtkwave dump.vcd
