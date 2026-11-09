-- ================================================================================ --
-- NEORV32 SoC - XBUS to Wishbone (B4 Pipelined) Bridge                             --
-- -------------------------------------------------------------------------------- --
-- The XBUS is NOT Wishbone-compatible: its STB is a single-cycle request pulse and --
-- NEORV32 cannot accept a response in the same cycle as its request. This bridge   --
-- provides a basic Wishbone B4 pipelined host interface (single transfers only,    --
-- one access at a time):                                                           --
--                                                                                  --
-- * STB is issued directly with the XBUS request and held while STALL is set.      --
-- * wb_stall_i has no default and has to be connected: tie it to '0' for a         --
--   pipelined device that never stalls. A classic device can be connected with     --
--   wb_stall_i <= not (wb_ack_i or wb_err_i or wb_rty_i), which holds STB until    --
--   the response (wait states supported).                                          --
-- * A response in the XBUS request cycle (asynchronous device) is registered       --
--   (ACK/ERR and read data) and forwarded one cycle later; a later response is     --
--   passed through directly. Access latency: 1 cycle for asynchronous responses    --
--   and for registered responses without wait states (pipelined or classic).       --
-- * Atomic read-modify-write (XBUS CTI = "001"): CYC stays asserted for the read   --
--   and the write single transfer and LOCK is asserted.                            --
-- * Burst transfers are NOT supported: the processor has to be configured with     --
--   CACHE_BURSTS_EN = false (caches then update blocks using single transfers).    --
--   Use the XBUS to AXI4 bridge for burst-capable setups (e.g. external memory).   --
--                                                                                  --
-- RTY is treated as an error. Dropping XBUS CYC (bus timeout) aborts any transfer. --
-- CYC is negated for at least one cycle after each access (except between read     --
-- and write of an atomic RMW) until the next request, as the XBUS keeps CYC        --
-- asserted across back-to-back accesses (bus switch arbitration, locked cache      --
-- block transfers) while some interconnects (e.g. the general-cores crossbar)      --
-- keep the device connection for the entire CYC. A request directly following a    --
-- passed-through response is therefore delayed by one cycle.                       --
--                                                                                  --
-- Cycle tag TGC (same encoding as AXI4 xPROT) is captured with the XBUS request    --
-- and stable while CYC is set (the XBUS tag of an instruction fetch can change     --
-- while the fetch is pending, e.g. on a privilege mode change):                    --
--   TGC(2): 0 = data access, 1 = instruction fetch                                 --
--   TGC(1): 0 = secure access (always zero)                                        --
--   TGC(0): 0 = unprivileged (user-mode) access, 1 = privileged access             --
--                                                                                  --
-- Simulation assertions check the XBUS behaviour this bridge relies on and the     --
-- Wishbone device handshake.                                                       --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) Stephan Nolting. All rights reserved.                              --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
--                                                                                  --
-- Assisted by Claude Code (Anthropic).                                             --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity xbus2wb_bridge is
  port (
    -- Global control --
    clk        : in  std_logic;
    resetn     : in  std_logic;
    -- XBUS device interface --
    xbus_req_i : in  xbus_req_t;
    xbus_rsp_o : out xbus_rsp_t;
    -- Wishbone host interface --
    wb_adr_o   : out std_logic_vector(31 downto 0); -- address
    wb_dat_o   : out std_logic_vector(31 downto 0); -- write data
    wb_dat_i   : in  std_logic_vector(31 downto 0); -- read data
    wb_we_o    : out std_logic;                     -- write enable
    wb_sel_o   : out std_logic_vector(3 downto 0);  -- byte select
    wb_cyc_o   : out std_logic;                     -- valid bus cycle
    wb_stb_o   : out std_logic;                     -- request strobe
    wb_lock_o  : out std_logic;                     -- locked (atomic read-modify-write) cycle
    wb_tgc_o   : out std_logic_vector(2 downto 0);  -- cycle tag: instruction/data, secure, privileged (see header)
    wb_stall_i : in  std_logic;                     -- request not accepted (classic device: see header)
    wb_ack_i   : in  std_logic;                     -- acknowledge
    wb_err_i   : in  std_logic;                     -- error
    wb_rty_i   : in  std_logic                      -- retry (treated as error)
  );
end entity;

architecture xbus2wb_bridge_rtl of xbus2wb_bridge is

  -- access state --
  type state_t is (
    S_IDLE,    -- no access in progress; CYC/STB follow the XBUS request
    S_STALLED, -- STB held: request stalled by the device or deferred by the CYC gap
    S_WAIT,    -- request accepted, waiting for the response
    S_CONT,    -- atomic RMW: read done, CYC held until the write request
    S_GAP      -- CYC negated for one cycle after the final response
  );
  signal state, state_nxt : state_t;

  -- Wishbone handshake --
  signal wb_stall, wb_ack, wb_err : std_ulogic;
  signal rmw, rmw_read, rmw_write, final_rsp : std_ulogic;

  -- Wishbone request --
  signal wb_cyc, wb_stb, direct : std_ulogic;
  signal tgc : std_ulogic_vector(2 downto 0);

  -- XBUS response --
  signal dly_ack, dly_err, rsp_ack, rsp_err : std_ulogic;
  signal dly_dat : std_ulogic_vector(31 downto 0);

begin

  -- Request Control ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_stall <= std_ulogic(wb_stall_i);
  wb_ack   <= std_ulogic(wb_ack_i);
  wb_err   <= std_ulogic(wb_err_i) or std_ulogic(wb_rty_i);

  -- final response of an access: any error, or an ACK of anything except the read of an
  -- atomic read-modify-write (the write follows within the same CYC) --
  rmw       <= bool_to_ulogic_f(xbus_req_i.cti = "001");
  rmw_read  <= rmw and (not xbus_req_i.we);
  rmw_write <= rmw and xbus_req_i.we;
  final_rsp <= wb_err or (wb_ack and (not rmw_read));

  request_control: process(state, xbus_req_i, wb_stall, wb_ack, final_rsp, rmw_write)
  begin
    -- defaults --
    state_nxt <= state;
    wb_cyc    <= '0';
    wb_stb    <= '0';

    case state is

      when S_IDLE => -- issue STB directly with the XBUS request
      -- ------------------------------------------------------------
        wb_cyc <= xbus_req_i.stb;
        wb_stb <= xbus_req_i.stb;
        if (xbus_req_i.stb = '1') then
          if (final_rsp = '1') then -- immediate response (delayed for the XBUS)
            state_nxt <= S_GAP;
          elsif (wb_ack = '1') then -- immediate response to RMW read
            state_nxt <= S_CONT;
          elsif (wb_stall = '1') then
            state_nxt <= S_STALLED;
          else
            state_nxt <= S_WAIT;
          end if;
        end if;

      when S_STALLED => -- hold STB until the request is accepted
      -- ------------------------------------------------------------
        wb_cyc <= '1';
        wb_stb <= '1';
        if (final_rsp = '1') then
          state_nxt <= S_GAP;
        elsif (wb_ack = '1') then -- response to RMW read
          state_nxt <= S_CONT;
        elsif (wb_stall = '0') then
          state_nxt <= S_WAIT;
        end if;

      when S_WAIT => -- wait for the response
      -- ------------------------------------------------------------
        wb_cyc <= '1';
        if (final_rsp = '1') then
          state_nxt <= S_GAP;
        elsif (wb_ack = '1') then -- response to RMW read
          state_nxt <= S_CONT;
        end if;

      when S_CONT => -- keep CYC between RMW read and write; issue the write STB directly
      -- ------------------------------------------------------------
        wb_cyc <= '1';
        wb_stb <= xbus_req_i.stb and rmw_write;
        if (xbus_req_i.stb = '1') then
          if (rmw_write = '0') or (final_rsp = '1') then -- not the RMW write (dropped, see checks) or immediate response
            state_nxt <= S_GAP;
          elsif (wb_stall = '1') then
            state_nxt <= S_STALLED;
          else
            state_nxt <= S_WAIT;
          end if;
        end if;

      when S_GAP => -- CYC negated; defer a request that follows directly
      -- ------------------------------------------------------------
        if (xbus_req_i.stb = '1') then
          state_nxt <= S_STALLED;
        else
          state_nxt <= S_IDLE;
        end if;

    end case;

    -- XBUS terminates the access (bus timeout) --
    if (xbus_req_i.cyc = '0') then
      state_nxt <= S_IDLE;
      wb_cyc    <= '0';
      wb_stb    <= '0';
    end if;
  end process;

  -- STB issued in the XBUS request cycle (S_IDLE, S_CONT): the XBUS cannot accept a response yet --
  direct <= bool_to_ulogic_f(state = S_IDLE or state = S_CONT) and wb_stb;

  state_reg: process(resetn, clk)
  begin
    if (resetn = '0') then
      state   <= S_IDLE;
      tgc     <= (others => '0');
      dly_ack <= '0';
      dly_err <= '0';
    elsif rising_edge(clk) then
      state <= state_nxt;
      -- cycle tag of the access; kept for the RMW write --
      if (xbus_req_i.stb = '1') and ((state = S_IDLE) or (state = S_GAP)) then
        tgc <= xbus_req_i.tag;
      end if;
      -- register a response that arrives in the XBUS request cycle --
      dly_ack <= direct and wb_ack;
      dly_err <= direct and wb_err;
    end if;
  end process;


  -- Response Control -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- read data register; only used while dly_ack is set, so no reset and no enable required --
  data_delay: process(clk)
  begin
    if rising_edge(clk) then
      dly_dat <= std_ulogic_vector(wb_dat_i);
    end if;
  end process;

  -- immediate response: forward one cycle later; later response: pass through --
  rsp_ack <= xbus_req_i.cyc and (dly_ack or (wb_ack and (not direct)));
  rsp_err <= xbus_req_i.cyc and (dly_err or (wb_err and (not direct)));

  xbus_rsp_o.ack  <= rsp_ack;
  xbus_rsp_o.err  <= rsp_err;
  xbus_rsp_o.data <= dly_dat when (dly_ack = '1') else std_ulogic_vector(wb_dat_i); -- 32-bit mux


  -- Wishbone Interface ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  wb_cyc_o  <= std_logic(wb_cyc);
  wb_stb_o  <= std_logic(wb_stb);
  wb_we_o   <= std_logic(xbus_req_i.we);
  wb_sel_o  <= std_logic_vector(xbus_req_i.sel);
  wb_adr_o  <= std_logic_vector(xbus_req_i.addr);
  wb_dat_o  <= std_logic_vector(xbus_req_i.data);
  wb_lock_o <= std_logic(wb_cyc and rmw); -- atomic RMW
  wb_tgc_o  <= std_logic_vector(xbus_req_i.tag) when (state = S_IDLE) else std_logic_vector(tgc);


  -- Simulation Checks ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- pragma translate_off
  sim_check: process(clk)
    variable req_v : xbus_req_t; -- XBUS request of the current access
  begin
    if rising_edge(clk) and (resetn = '1') then

      -- XBUS: request encoding --
      assert not ((xbus_req_i.stb = '1') and (xbus_req_i.cyc = '0')) report
        "[NEORV32] xbus2wb_bridge: XBUS STB without CYC." severity error;
      assert (xbus_req_i.cyc = '0') or (xbus_req_i.cti /= "010") report
        "[NEORV32] xbus2wb_bridge: burst transfers are not supported - set CACHE_BURSTS_EN = false." severity error;
      assert (xbus_req_i.cyc = '0') or (xbus_req_i.cti = "000") or (xbus_req_i.cti = "001") or (xbus_req_i.cti = "010") report
        "[NEORV32] xbus2wb_bridge: unknown XBUS CTI." severity error;

      -- XBUS: one access at a time; a new request only after the response has been delivered --
      assert not ((xbus_req_i.stb = '1') and ((state = S_STALLED) or (state = S_WAIT))) report
        "[NEORV32] xbus2wb_bridge: XBUS request while an access is pending." severity error;
      assert not ((xbus_req_i.stb = '1') and ((rsp_ack or rsp_err) = '1')) report
        "[NEORV32] xbus2wb_bridge: XBUS request in the cycle of its previous response." severity error;

      -- XBUS: request stable while the access is pending (write data only for writes) --
      if ((state = S_STALLED) or (state = S_WAIT)) and (xbus_req_i.cyc = '1') then
        assert (xbus_req_i.addr = req_v.addr) and (xbus_req_i.we = req_v.we) and (xbus_req_i.sel = req_v.sel) and
               (xbus_req_i.cti = req_v.cti) and ((req_v.we = '0') or (xbus_req_i.data = req_v.data)) report
          "[NEORV32] xbus2wb_bridge: XBUS request changed while the access is pending." severity error;
      end if;

      -- XBUS: atomic RMW = read immediately followed by a write to the same location within the lock --
      assert not ((xbus_req_i.stb = '1') and (rmw_write = '1') and (state /= S_CONT)) report
        "[NEORV32] xbus2wb_bridge: XBUS RMW write without preceding RMW read." severity error;
      assert not ((xbus_req_i.stb = '1') and (rmw_write = '0') and (state = S_CONT)) report
        "[NEORV32] xbus2wb_bridge: XBUS request other than the RMW write after an RMW read (dropped)." severity error;
      if (state = S_CONT) and (xbus_req_i.cyc = '1') then
        assert (xbus_req_i.cti = "001") and (xbus_req_i.addr = req_v.addr) and (xbus_req_i.sel = req_v.sel) and
               (xbus_req_i.tag = req_v.tag) report
          "[NEORV32] xbus2wb_bridge: XBUS lock, address, byte enable or tag changed between RMW read and write." severity error;
      end if;

      -- Wishbone device: handshake; only one request is pending at a time, so a response while STB is
      -- stalled cannot belong to an earlier (accepted) request --
      assert not ((wb_ack and wb_err) = '1') report
        "[NEORV32] xbus2wb_bridge: Wishbone ACK and ERR/RTY at the same time." severity error;
      assert not (((wb_ack or wb_err) = '1') and (wb_stb = '1') and (wb_stall = '1')) report
        "[NEORV32] xbus2wb_bridge: Wishbone response to a request that was not accepted (STB stalled, no other request pending)." severity error;
      assert not (((wb_ack or wb_err) = '1') and (wb_stb = '0') and (state /= S_WAIT)) report
        "[NEORV32] xbus2wb_bridge: Wishbone response without pending request." severity error;

      -- capture the request of the next access --
      if (xbus_req_i.stb = '1') then
        req_v := xbus_req_i;
      end if;

    end if;
  end process;
  -- pragma translate_on

end architecture;
