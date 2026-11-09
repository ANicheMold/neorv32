## Processor System Integration

### > `neorv32_litex_core_complex.vhd`

Pre-configured top entity wrapper for integration within the [LiteX](https://github.com/enjoy-digital/litex)
SoC builder framework. This wrapper provides AXI4 and AXI4-Stream-compatible interfaces.

> [!TIP]
> See the user guide section [`core/mem`](https://stnolting.github.io/neorv32/ug/#_litex_soc_builder_support)
for more information.

> [!NOTE]
> The provided top entity wrapper can also be used for custom (AXI) setups outside of Vivado IP block designs.


### > `neorv32_vivado_ip.vhd`

Processor top entity with optional AXI4 and AXI4-Stream interfaces. Dedicated for integration as custom
IP block within AMD Vivado. Run the provided packaging script in the Vivado TCL shell to generate a NEORV32
IP block:

```tcl
source neorv32_vivado_ip.tcl
```

This wrapper uses the `xbus2axi4_bridge.vhd` to convert the processor's XBUS protocol into the AXI4 protocol.

> [!TIP]
> See the user guide's [UG: Packaging the Processor as Vivado IP Block](https://stnolting.github.io/neorv32/ug/#_packaging_the_processor_as_vivado_ip_block)
section for more information and step-by-step instructions for generating a NEORV32 IP module.



### > `xbus2wb_bridge.vhd`

Converts the processor's XBUS protocol into a basic Wishbone B4 **pipelined** host interface
(single transfers only, one access at a time). The XBUS uses a single-cycle `STB` pulse, so it
cannot be connected to a Wishbone device directly, and NEORV32 cannot accept a response in the same
cycle as its request.

* `STB` is issued directly with the XBUS request and held while `wb_stall_i` is set.
* `wb_stall_i` has no default and has to be connected: tie it to `'0'` for a pipelined device that
  never stalls. A classic device can be connected with
  `wb_stall_i <= not (wb_ack_i or wb_err_i or wb_rty_i)`, which holds `STB` until the response
  (wait states supported).
* A response in the XBUS request cycle (asynchronous device) is registered (`ACK`/`ERR` + read data)
  and forwarded one cycle later; a later response is passed through. Access latency: 1 cycle for
  asynchronous responses and for registered responses without wait states (pipelined or classic).
  The response register costs 34 flip-flops and a 32-bit read-data mux.
  It is needed because the processor cannot accept a response in the cycle of its request: the XBUS
  gateway (`neorv32_xbus.vhd`) only passes a response while its `pending` flag is set, which happens
  one cycle after `STB`, and the AMO unit (`neorv32_bus_amo_rmw` in `neorv32_bus.vhd`) ignores an
  `ACK` in the cycle in which it issues the RMW write. If a future core version accepts same-cycle
  responses, the response register can be removed; holding `STB`, the `CYC` handling and the `TGC`
  capture are still needed.
* Atomic read-modify-write: `CYC` kept asserted for read and write with `LOCK` set.
* Cycle tag: `wb_tgc_o` uses the same encoding as AXI4 `xPROT` (bit 2: instruction fetch, bit 1:
  non-secure access, always 0 = secure, bit 0: privileged access). It is captured with the XBUS
  request and stable while `CYC` is set (the XBUS tag of a pending instruction fetch can change, e.g.
  on a privilege mode change).
* `CYC` is negated between independent accesses: the XBUS keeps `CYC` asserted across back-to-back
  accesses (bus switch arbitration, locked cache block transfers), while some interconnects (e.g. the
  general-cores crossbar) keep the device connection for the entire `CYC`. A request directly
  following a passed-through response is therefore delayed by one cycle.
* `RTY` is treated as an error.
* Simulation assertions check the XBUS behaviour the bridge relies on (one access at a time, no
  request in the cycle of a response, stable request while pending, RMW read/write sequence,
  supported `CTI`) and the Wishbone device handshake.

> [!IMPORTANT]
> Burst transfers are not supported: configure the processor with `CACHE_BURSTS_EN = false`
> (caches then update their blocks using single transfers). A simulation assertion flags burst
> requests. Use `xbus2axi4_bridge.vhd` for burst-capable setups such as external memory.

> [!NOTE]
> Assisted by [Claude Code](https://claude.com/claude-code) (Anthropic).
