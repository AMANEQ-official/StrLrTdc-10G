library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_MISC.ALL;
use ieee.numeric_std.all;

library UNISIM;
use UNISIM.VComponents.all;

library mylib;
use mylib.defDCR.all;
use mylib.defBCT.all;
use mylib.defBusAddressMap.all;
use mylib.defCDCM.all;
use mylib.defMikumari.all;
use mylib.defMikumariUtil.all;
use mylib.defLaccp.all;
use mylib.defHeartBeatUnit.all;
use mylib.defFreeRunScaler.all;
use mylib.defSiTCP_XG.all;
use mylib.defRBCP.all;

use mylib.defDataBusAbst.all;
use mylib.defDelimiter.all;
use mylib.defTDC.all;
use mylib.defStrLRTDC.all;
use mylib.defIOManager.all;

entity toplevel is
  Port (
    -- System ---------------------------------------------------------------
    PROGB_ON            : out std_logic;
    BASE_CLKP           : in std_logic;
    BASE_CLKN           : in std_logic;
    USR_RSTB            : in std_logic;
    LED                 : out std_logic_vector(4 downto 1);
    DIP                 : in std_logic_vector(4 downto 1);
    VP                  : in std_logic;
    VN                  : in std_logic;

-- GTX ------------------------------------------------------------------
    GTX_REFCLK_P        : in std_logic;
    GTX_REFCLK_N        : in std_logic;
    GTX_TX_P            : out std_logic;
    GTX_RX_P            : in  std_logic;
    GTX_TX_N            : out std_logic;
    GTX_RX_N            : in  std_logic;

-- SPI flash ------------------------------------------------------------
    MOSI                : out std_logic;
    DIN                 : in std_logic;
    FCSB                : out std_logic;

-- MIKUMARI connector ---------------------------------------------------
    MIKUMARI_RXP             : in std_logic;
    MIKUMARI_RXN             : in std_logic;
    MIKUMARI_TXP             : out std_logic;
    MIKUMARI_TXN             : out std_logic;

-- EEPROM ---------------------------------------------------------------
    EEP_CS              : out std_logic_vector(2 downto 1);
    EEP_SK              : out std_logic_vector(2 downto 1);
    EEP_DI              : out std_logic_vector(2 downto 1);
    EEP_DO              : in std_logic_vector(2 downto 1);

-- NIM-IO ---------------------------------------------------------------
    NIM_IN              : in std_logic_vector(2 downto 1);
    NIM_OUT             : out std_logic_vector(2 downto 1);

-- JItter cleaner -------------------------------------------------------
    CDCE_PDB            : out std_logic;
--    CDCE_LOCK           : in std_logic;
--    CDCE_SCLK           : out std_logic;
--    CDCE_SO             : in std_logic;
--    CDCE_SI             : out std_logic;
--    CDCE_LE             : out std_logic;
--    CDCE_REFP           : out std_logic;
--    CDCE_REFN           : out std_logic;
--
--    CLK_FASTP           : in std_logic;
--    CLK_FASTN           : in std_logic;
--    CLK_SLOWP           : in std_logic;
--    CLK_SLOWN           : in std_logic;
--
-- Main port ------------------------------------------------------------
-- Up port --
    MAIN_IN_U           : in std_logic_vector(31 downto 0);
-- Down port --
    MAIN_IN_D           : in std_logic_vector(31 downto 0);

-- Mezzanine slot -------------------------------------------------------
-- Up slot --
    MZN_UP              : in std_logic_vector(31 downto 0);
    MZN_UN              : in std_logic_vector(31 downto 0);

-- Dwon slot --
    MZN_DP              : in std_logic_vector(31 downto 0);
    MZN_DN              : in std_logic_vector(31 downto 0)

-- DDR3 SDRAM -----------------------------------------------------------

  );
end toplevel;

architecture Behavioral of toplevel is
  attribute mark_debug : string;
  constant kEnDebugTop  : string:= "false";

  -- System --------------------------------------------------------------------------------
  -- AMANEQ specification
  constant kNumLED      : integer:= 4;
  constant kNumBitDIP   : integer:= 4;
  constant kNumNIM      : integer:= 2;
  constant kNumGtx      : integer:= 1;
  constant kNumInputMZN : integer:= 32;

  signal async_reset  : std_logic;
  signal pwr_on_reset : std_logic;
  signal system_reset : std_logic;
  signal system_reset_xgmii : std_logic;
  signal user_reset   : std_logic;
  signal hbu_reset    : std_logic;

  signal core_master_reset  : std_logic;

  signal emergency_reset  : std_logic;
  signal bct_reset    : std_logic;
  signal rst_from_bus : std_logic;
  signal delayed_usr_rstb : std_logic;
  signal rst_from_miku_xgmii    : std_logic;

  attribute keep                      : string;
  attribute keep of user_reset        : signal is "true";
  attribute keep of bct_reset         : signal is "true";
  attribute keep of emergency_reset   : signal is "true";

  signal sync_nim_in      : std_logic_vector(NIM_IN'range);
  signal tmp_nim_out      : std_logic_vector(NIM_OUT'range);

  signal local_frame_flag : std_logic_vector(kWidthFrameFlag-1 downto 0);
  signal frame_flag_out   : std_logic_vector(kWidthFrameFlag-1 downto 0);

  signal local_trigger_in : std_logic;

  -- Hit Input definition --
  constant kNumInput    : integer:= 128;

  -- DIP -----------------------------------------------------------------------------------
  signal dip_sw       : std_logic_vector(DIP'range);
  subtype DipID is integer range 0 to 4;
  type regLeaf is record
    Index : DipID;
  end record;
  constant kSiTCP       : regLeaf := (Index => 1);
  constant kTriggerOut  : regLeaf := (Index => 2);
  constant kStandAlone  : regLeaf := (Index => 3);
  constant kNC4         : regLeaf := (Index => 4);
  constant kDummy       : regLeaf := (Index => 0);

  -- Mezzanine ----------------------------------------------------------------------------
  -- DCR --
  signal mzn_u        : std_logic_vector(kNumInputMZN-1 downto 0);
  signal mzn_d        : std_logic_vector(kNumInputMZN-1 downto 0);
  signal dcr_u        : std_logic_vector(kNumInputMZN-1 downto 0);
  signal dcr_d        : std_logic_vector(kNumInputMZN-1 downto 0);

  -- MIKUMARI -----------------------------------------------------------------------------
  constant  kPcbVersion : string:= "GN-2006-4";
  --constant  kPcbVersion : string:= "GN-2006-1";

  function GetMikuIoStd(version: string) return string is
  begin
    case version is
      when  "GN-2006-4" => return "LVDS";
      when others       => return "LVDS_25";
    end case;
  end function;

  constant kNumMikumari       : integer:= 1;
  constant kIdMikuSec         : integer:= 0;

  signal miku_txp, miku_txn, miku_rxp, miku_rxn   : std_logic_vector(kNumMikumari-1 downto 0);

  subtype MikuScalarPort is std_logic_vector(kNumMikumari-1 downto 0);

  -- CDCM --
  signal power_on_init        : std_logic;

  signal cbt_lane_up          : MikuScalarPort;
  signal pattern_error        : MikuScalarPort;
  signal watchdog_error       : MikuScalarPort;

  signal mod_clk, gmod_clk    : std_logic;

  --type TapValueArray  is array(kNumMikumari-1 downto 0) of std_logic_vector(kWidthTap-1 downto 0);
  --type SerdesOffsetArray is array(kNumMikumari-1 downto 0) of signed(kWidthSerdesOffset-1 downto 0);
  signal tap_value_out        : TapArrayType(kNumMikumari-1 downto 0);
  signal bitslip_num_out      : BitslipArrayType(kNumMikumari-1 downto 0);
  signal serdes_offset        : SerdesOfsArrayType(kNumMikumari-1 downto 0);

  -- Mikumari --
  type MikuDataArray is array(kNumMikumari-1 downto 0) of std_logic_vector(7 downto 0);
  type MikuPulseTypeArray is array(kNumMikumari-1 downto 0) of MikumariPulseType;

  signal miku_tx_ack        : MikuScalarPort;
  signal miku_data_tx       : MikuDataArray;
  signal miku_valid_tx      : MikuScalarPort;
  signal miku_last_tx       : MikuScalarPort;
  signal busy_pulse_tx      : MikuScalarPort;

  signal mikumari_link_up   : MikuScalarPort;
  signal miku_data_rx       : MikuDataArray;
  signal miku_valid_rx      : MikuScalarPort;
  signal miku_last_rx       : MikuScalarPort;
  signal checksum_err       : MikuScalarPort;
  signal frame_broken       : MikuScalarPort;
  signal recv_terminated    : MikuScalarPort;

  signal pulse_tx, pulse_rx : MikuScalarPort;
  signal pulse_type_tx, pulse_type_rx  : MikuPulseTypeArray;

 -- LACCP --
  signal laccp_reset        : MikuScalarPort;
  signal laccp_pulse_out    : std_logic_vector(kNumLaccpPulse-1 downto 0);

  signal is_ready_for_daq   : MikuScalarPort;
  signal sync_pulse_out     : std_logic;

  signal is_ready_laccp_intra   : std_logic_vector(kNumExtIntraPort-1 downto 0);
  signal valid_laccp_intra_in   : std_logic_vector(kNumExtIntraPort-1 downto 0);
  signal valid_laccp_intra_out  : std_logic_vector(kNumExtIntraPort-1 downto 0);
  signal data_laccp_intra_in    : ExtIntraType;
  signal data_laccp_intra_out   : ExtIntraType;

  -- RLIGP --
  --type LinkAddrArray is array(kNumMikumari-1 downto 0) of std_logic_vector(kPosRegister'range);

  signal link_addr_partter  : IpAddrArrayType(kNumMikumari-1 downto 0);
  signal valid_link_addr    : MikuScalarPort;

  -- RCAP --
--  signal idelay_tap_in      : unsigned(tap_value_out'range);

  signal valid_hbc_offset   : std_logic;
  signal hbc_offset         : std_logic_vector(kWidthHbCount-1 downto 0);
  signal laccp_fine_offset  : signed(kWidthLaccpFineOffset-1 downto 0);
  signal local_fine_offset  : signed(kWidthLaccpFineOffset-1 downto 0);

  -- Heartbeat --
  signal hbu_is_synchronized  : std_logic;
  signal heartbeat_signal   : std_logic;
  signal heartbeat_count    : std_logic_vector(kWidthHbCount-1 downto 0);
  signal hbf_number         : std_logic_vector(kWidthHbfNum-1 downto 0);
  signal hbf_state          : HbfStateType;
  signal frame_ctrl_gate    : std_logic;
  signal hbf_num_mismatch   : std_logic;

  attribute mark_debug of is_ready_for_daq   : signal is kEnDebugTop;
  attribute mark_debug of sync_pulse_out     : signal is kEnDebugTop;
  attribute mark_debug of serdes_offset      : signal is kEnDebugTop;
  attribute mark_debug of laccp_fine_offset  : signal is kEnDebugTop;
  attribute mark_debug of local_fine_offset  : signal is kEnDebugTop;

  -- Mikumari Util ------------------------------------------------------------
  signal cbt_init_from_mutil  : MikuScalarPort;
  signal rst_from_miku        : std_logic;

  -- Scaler -------------------------------------------------------------------
  constant kMsbScr      : integer:= kNumSysInput+kNumInput-1;
  signal scr_en_in      : std_logic_vector(kMsbScr downto 0):= (others => '0');
  signal scr_gate       : std_logic_vector(kNumScrGate-1 downto 0);

  -- Streaming TDC ------------------------------------------------------------
  -- scaler --
  constant kNumScrThr   : integer:= 5;
  signal hit_out        : std_logic_vector(127 downto 0):= (others => '0');
  signal scr_thr_on     : std_logic_vector(kNumScrThr-1 downto 0);
  signal daq_is_runnig  : std_logic;

  signal signal_in_merge    : std_logic_vector(kNumInput-1 downto 0);
  signal strtdc_trigger_in  : std_logic;

  -- VitalBlock output --
  signal vital_rden         : std_logic;
  signal vital_dout         : std_logic_vector(kWidthData-1 downto 0);
  signal vital_empty        : std_logic;
  signal vital_almost_empty : std_logic;
  signal vital_valid        : std_logic;

  -- Link buffer --
  signal din_link_buf       : std_logic_vector(kWidthData-1 downto 0);
  signal pfull_link_buf     : std_logic;
  signal full_link_buf      : std_logic;

  --attribute mark_debug of vital_valid : signal is "true";
  --attribute mark_debug of pfull_link_buf : signal is "true";
  --attribute mark_debug of full_link_buf : signal is "true";

    COMPONENT LinkBuffer
  PORT (
    rst : IN STD_LOGIC;
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC;
    valid : OUT STD_LOGIC;
    prog_full : OUT STD_LOGIC
--    wr_rst_busy : OUT STD_LOGIC;
--    rd_rst_busy : OUT STD_LOGIC
  );
  END COMPONENT;


  -- IOM ----------------------------------------------------------------------------------
  signal intsig_from_iom        : std_logic_vector(3 downto 0);
  signal intsig_to_iom          : std_logic_vector(7 downto 0);

  -- C6C ----------------------------------------------------------------------------------
  signal c6c_reset              : std_logic;
  signal c6c_fast, c6c_slow     : std_logic;

  -- MIG ----------------------------------------------------------------------------------

  -- SDS ---------------------------------------------------------------------
  signal shutdown_over_temp     : std_logic;
  signal uncorrectable_flag     : std_logic;

  -- FMP ---------------------------------------------------------------------

  -- BCT -----------------------------------------------------------------------------------
  signal addr_LocalBus          : LocalAddressType;
  signal data_LocalBusIn        : LocalBusInType;
  signal data_LocalBusOut       : DataArray;
  signal re_LocalBus            : ControlRegArray;
  signal we_LocalBus            : ControlRegArray;
  signal ready_LocalBus         : ControlRegArray;

  -- TSD -----------------------------------------------------------------------------------
  signal wd_to_tsd                              : std_logic_vector(kWidthDaqData-1 downto 0);
  signal we_to_tsd, empty_to_tsd, re_from_tsd   : std_logic;

  -- SiTCP ---------------------------------------------------------------------------------
  signal sitcp_ip_addr  : std_logic_vector(31 downto 0);

  signal tcp_is_active, close_req, close_ack    : std_logic;

  signal tcp_tx_afull : std_logic;
  signal tcp_txb      : std_logic_vector(kNumByteTCPXG'range);
  signal tcp_txd      : std_logic_vector(kWidthDataXGMII-1 downto 0);

  signal tcp_rx_clr_enb   : std_logic;
  signal tcp_rx_wadr      : std_logic_vector(kWidthAddrRx-1 downto 0);

  signal rbcp_addr    : std_logic_vector(kWidthAddrRBCP-1 downto 0);
  signal rbcp_wd      : std_logic_vector(kWidthDataRBCP-1 downto 0);
  signal rbcp_we      : std_logic;
  signal rbcp_re      : std_logic;
  signal rbcp_ack     : std_logic;
  signal rbcp_rd      : std_logic_vector(kWidthDataRBCP-1 downto 0);

  signal rbcp_xg_addr    : std_logic_vector(kWidthAddrRBCP-1 downto 0);
  signal rbcp_xg_wd      : std_logic_vector(kWidthDataRBCP-1 downto 0);
  signal rbcp_xg_we      : std_logic;
  signal rbcp_xg_re      : std_logic;
  signal rbcp_xg_ack     : std_logic;
  signal rbcp_xg_rd      : std_logic_vector(kWidthDataRBCP-1 downto 0);

  component WRAP_SiTCPXG_XC7K_128K
    generic(
      RxBufferSize	: string
      );
    port(
      REG_FPGA_VER              : in std_logic_vector(31 downto 0);
      REG_FPGA_ID               : in std_logic_vector(31 downto 0);

      --		==== System I/F ====
      FORCE_DEFAULTn            : in std_logic;
      XGMII_CLOCK               : in std_logic;
      RSTs                      : in std_logic;

      --		==== XGMII I/F ====
      XGMII_RXC 								: in std_logic_vector(7 downto 0);
      XGMII_RXD 								: in std_logic_vector(63 downto 0);
      XGMII_TXC									: out std_logic_vector(7 downto 0);
			XGMII_TXD									: out std_logic_vector(63 downto 0);

      --		==== 93C46 I/F ====
			EEPROM_CS									: out std_logic;
			EEPROM_SK									: out std_logic;
			EEPROM_DI									: out std_logic;
      EEPROM_DO									: in std_logic;

      --		==== User I/F ====
      SiTCP_RESET_OUT	          : out std_logic;
      IP_ADDR                   : out std_logic_vector(31 downto 0);
      --		--- RBCP ---
			RBCP_ACT                  : out std_logic;
			RBCP_ADDR                 : out std_logic_vector(31 downto 0);
			RBCP_WE										: out std_logic;
			RBCP_WD										: out std_logic_vector(7 downto 0);
			RBCP_RE										: out std_logic;
      RBCP_ACK                  : in std_logic;
			RBCP_RD	                  : in std_logic_vector(7 downto 0);
      -- --- TCP ---
      USER_SESSION_OPEN_REQ	    : in std_logic;
		 	USER_SESSION_ESTABLISHED  : out std_logic;
		 	USER_SESSION_CLOSE_REQ    : out std_logic;
      USER_SESSION_CLOSE_ACK    : in std_logic;
      USER_TX_D                 : in std_logic_vector(63 downto 0);
      USER_TX_B	                : in std_logic_vector(3 downto 0);
		 	USER_TX_AFULL             : out std_logic;
      USER_RX_SIZE              : in std_logic_vector(15 downto 0);
		 	USER_RX_CLR_ENB	          : out std_logic;
      USER_RX_CLR_REQ           : in std_logic;
      USER_RX_RADR 							: in std_logic_vector(15 downto 0);
      USER_RX_WADR 							: out std_logic_vector(15 downto 0);
      USER_RX_WENB 							: out std_logic_vector(7 downto 0);
      USER_RX_WDAT 							: out std_logic_vector(63 downto 0)
      );
  end component;

  -- SFP transceiver -----------------------------------------------------------------------
  constant kPcsBlockLock      : integer:= 0;
  signal pcs_pma_status       : std_logic_vector(7 downto 0);

  --  constant kMiiPhyad      : std_logic_vector(kWidthPhyAddr-1 downto 0):= "00000";
  --  signal mii_init_mdc, mii_init_mdio : std_logic;

  signal clk_xgmii        : std_logic;
  signal qpll_locked      : std_logic;

  signal xgmii_rxc        : std_logic_vector(kWidthCtrlXGMII-1 downto 0);
  signal xgmii_rxd        : std_logic_vector(kWidthDataXGMII-1 downto 0);
  signal xgmii_txc        : std_logic_vector(kWidthCtrlXGMII-1 downto 0);
  signal xgmii_txd        : std_logic_vector(kWidthDataXGMII-1 downto 0);

  -- DRP --
  signal drp_req          : std_logic;
  signal drp_den, drp_dwe, drp_drdy : std_logic;
  signal drp_addr         : std_logic_vector(15 downto 0);
  signal drp_di           : std_logic_vector(15 downto 0);
  signal drp_drpdo        : std_logic_vector(15 downto 0);

  COMPONENT ten_gig_eth_pcs_pma
    PORT (
      -- Core block --
      refclk_p              : IN STD_LOGIC;
      refclk_n              : IN STD_LOGIC;
      reset                 : IN STD_LOGIC;
      resetdone_out         : OUT STD_LOGIC;
      coreclk_out           : OUT STD_LOGIC;
      rxrecclk_out          : OUT STD_LOGIC;

      -- Tranceiver --
      txp 									: OUT STD_LOGIC;
      txn 									: OUT STD_LOGIC;
      rxp 									: IN STD_LOGIC;
      rxn 									: IN STD_LOGIC;

      -- SFP+ IF --
      signal_detect         : IN STD_LOGIC;
      tx_disable            : OUT STD_LOGIC;
      tx_fault              : IN STD_LOGIC;

      -- XGMII --
      xgmii_txd             : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
      xgmii_txc             : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
      xgmii_rxd             : OUT STD_LOGIC_VECTOR(63 DOWNTO 0);
      xgmii_rxc             : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);

      -- MDIO --
      mdc                   : IN STD_LOGIC;
      mdio_in               : IN STD_LOGIC;
      mdio_out              : OUT STD_LOGIC;
      mdio_tri              : OUT STD_LOGIC;
      prtad                 : IN STD_LOGIC_VECTOR(4 DOWNTO 0);

      -- DRP IF --
      dclk                  : IN STD_LOGIC;
      drp_req               : OUT STD_LOGIC;
      drp_gnt               : IN STD_LOGIC;

      drp_den_o             : OUT STD_LOGIC;
      drp_dwe_o             : OUT STD_LOGIC;
      drp_daddr_o           : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
      drp_di_o              : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
      drp_drdy_o            : OUT STD_LOGIC;
      drp_drpdo_o           : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
      drp_den_i             : IN STD_LOGIC;
      drp_dwe_i             : IN STD_LOGIC;
      drp_daddr_i           : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
      drp_di_i              : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
      drp_drdy_i            : IN STD_LOGIC;
      drp_drpdo_i           : IN STD_LOGIC_VECTOR(15 DOWNTO 0);

      -- mics --
      pma_pmd_type          : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
      core_status           : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      sim_speedup_control   : IN STD_LOGIC;

      txusrclk_out          : OUT STD_LOGIC;
      txusrclk2_out         : OUT STD_LOGIC;
      areset_datapathclk_out : OUT STD_LOGIC;
      gttxreset_out         : OUT STD_LOGIC;
      gtrxreset_out         : OUT STD_LOGIC;
      txuserrdy_out         : OUT STD_LOGIC;
      reset_counter_done_out : OUT STD_LOGIC;
      qplllock_out          : OUT STD_LOGIC;
      qplloutclk_out        : OUT STD_LOGIC;
      qplloutrefclk_out     : OUT STD_LOGIC
      );
  END COMPONENT;


  -- Clock ---------------------------------------------------------------------------
  signal clk_gbe, clk_sys   : std_logic;
  signal clk_locked         : std_logic;
  signal clk_sys_locked     : std_logic;
  signal clk_miku_locked    : std_logic;
  signal clk_spi            : std_logic;


  component clk_wiz_sys
    port
      (-- Clock in ports
        -- Clock out ports
        clk_sys          : out    std_logic;
        clk_indep_gtx    : out    std_logic;
        clk_spi          : out    std_logic;
--        clk_buf          : out    std_logic;
        -- Status and control signals
        reset            : in     std_logic;
        locked           : out    std_logic;
        clk_in1_p        : in     std_logic;
        clk_in1_n        : in     std_logic
        );
  end component;

  component mmcm_cdcm
    port
     (-- Clock in ports
      clk_in2           : in     std_logic;
      clk_in_sel           : in     std_logic;
      -- Clock out ports
      clk_slow          : out    std_logic;
      clk_fast          : out    std_logic;
      clk_tdc0          : out    std_logic;
      clk_tdc1          : out    std_logic;
      clk_tdc2          : out    std_logic;
      clk_tdc3          : out    std_logic;
      -- Status and control signals
      reset             : in     std_logic;
      locked            : out    std_logic;
      clk_in1           : in     std_logic
     );
    end component;

  signal clk_fast, clk_slow   : std_logic;
  signal clk_tdc              : std_logic_vector(kNumTdcClock-1 downto 0);
  signal clk_sys_div16        : std_logic;
  signal mmcm_cdcm_locked     : std_logic;
  signal mmcm_cdcm_reset      : std_logic;
  --signal pll_is_locked        : std_logic;


 begin
  -- ===================================================================================
  -- body
  -- ===================================================================================

  -- Global ----------------------------------------------------------------------------
  async_reset   <= not USR_RSTB;

  u_DelayUsrRstb : entity mylib.DelayGen
    generic map(kNumDelay => 128)
    port map(clk_sys, USR_RSTB, delayed_usr_rstb);

  --clk_miku_locked <= CDCE_LOCK;
  clk_miku_locked <= mmcm_cdcm_locked;
  clk_locked      <= clk_sys_locked and clk_miku_locked;

  --c6c_reset       <= (not clk_sys_locked) or (not delayed_usr_rstb);
  c6c_reset       <= '1';
  mmcm_cdcm_reset <= (not delayed_usr_rstb);

  pwr_on_reset      <= core_master_reset;
  u_SysReset : entity mylib.SystemReset
    generic map(
      kWidthResetSync => 32
    )
    port map(
      -- Seed of the master reset --
      asyncReset    => async_reset,
      mmcmLocked    => clk_locked,
      -- Async reset to 10GbE_PCSPMA --
      masterReset   => core_master_reset,
      qpllLocked    => qpll_locked,

      -- System reset --
      clkXgmii        => clk_xgmii,
      clkSys          => clk_slow,
      syncXgmiiReset  => system_reset_xgmii,
      syncSysReset    => system_reset
      );

  u_RstFromMiku : entity mylib.SigStretcher
    generic map(kLength => 8)
    port map(clk_slow, laccp_pulse_out(kDownPulseSysRst), rst_from_miku);

  user_reset      <= system_reset or rst_from_bus or rst_from_miku;
  bct_reset       <= system_reset or rst_from_miku;

  -- NIM input --
  u_sync_nimin1 : entity mylib.synchronizer port map(clk_slow, NIM_IN(1), sync_nim_in(1));
  u_sync_nimin2 : entity mylib.synchronizer port map(clk_slow, NIM_IN(2), sync_nim_in(2));

  -- NIM output --
  u_nimo_buf : process(clk_slow)
  begin
    if(clk_slow'event and clk_slow = '1') then
      NIM_OUT <= tmp_nim_out;
    end if;
  end process;

--  tmp_nim_out(1)  <= laccp_pulse_out(kDownPulseTrigger) when(dip_sw(kTriggerOut.Index) = '1') else heartbeat_signal;
--  tmp_nim_out(2)  <= tcp_is_active(0);

  dip_sw(1)   <= DIP(1);
  dip_sw(2)   <= DIP(2);
  dip_sw(3)   <= DIP(3);
  dip_sw(4)   <= DIP(4);

  LED         <= clk_miku_locked & mikumari_link_up(kIdMikuSec) & is_ready_for_daq & daq_is_runnig;

  -- Mezzanine connection --------------------------------------------------------------
  MIKUMARI_TXP  <= miku_txp(kIdMikuSec);
  MIKUMARI_TXN  <= miku_txn(kIdMikuSec);
  miku_rxp(kIdMikuSec)  <= MIKUMARI_RXP;
  miku_rxn(kIdMikuSec)  <= MIKUMARI_RXN;

  -- DCR
  gen_dcr : for i in 0 to kNumInputMZN-1 generate
    dcr_u_IBUFDS_inst : IBUFDS
      generic map (
        DIFF_TERM     => TRUE,  -- Differential Termination
        IBUF_LOW_PWR  => FALSE, -- Low power (TRUE) vs. performance (FALSE) setting for referenced I/O standards
        IOSTANDARD    => DcrUIoStd(i)
      )port map (
        O   => mzn_u(i),  -- Buffer output
        I   => MZN_UP(i), -- Diff_p buffer input (connect directly to top-level port)
        IB  => MZN_UN(i)  -- Diff_n buffer input (connect directly to top-level port)
      );
   dcr_d_IBUFDS_inst : IBUFDS
      generic map (
        DIFF_TERM     => TRUE,  -- Differential Termination
        IBUF_LOW_PWR  => FALSE, -- Low power (TRUE) vs. performance (FALSE) setting for referenced I/O standards
        IOSTANDARD    => DcrDIoStd(i)
      )port map (
        O   => mzn_d(i),  -- Buffer output
        I   => MZN_DP(i), -- Diff_p buffer input (connect directly to top-level port)
        IB  => MZN_DN(i)  -- Diff_n buffer input (connect directly to top-level port)
      );
  end generate;

  u_DCR_NetAssign: entity mylib.DCR_NetAssign
  port map(
    mznInU  => mzn_u,
    mznInD  => mzn_d,
    dcrOutU => dcr_u,
    dcrOutD => dcr_d
  );

  -- MIKUMARI --------------------------------------------------------------------------
  u_KeepInit : entity mylib.RstDelayTimer
    port map(system_reset, X"0FFFFFFF", clk_slow, open, power_on_init );

  gen_mikumari : for i in 0 to kNumMikumari-1 generate
    laccp_reset(i) <= system_reset or (not mikumari_link_up(i));

    u_Miku_Inst : entity mylib.MikumariBlock
      generic map(
        -- CBT generic -------------------------------------------------------------
        -- CDCM-Mod-Pattern --
        kCdcmModWidth    => 8,
        -- CDCM-TX --
        kIoStandardTx    => GetMikuIoStd(kPcbVersion),
        kTxPolarity      => FALSE,
        -- CDCM-RX --
        genIDELAYCTRL    => TRUE,
        kDiffTerm        => TRUE,
        kIoStandardRx    => GetMikuIoStd(kPcbVersion),
        kRxPolarity      => FALSE,
        kIoDelayGroup    => "idelay_1",
        kFixIdelayTap    => FALSE,
        kFreqFastClk     => 500.0,
        kFreqRefClk      => 200.0,
        -- Encoder/Decoder
        kNumEncodeBits   => 1,
        -- Master/Slave
        kCbtMode         => "Slave",
        -- DEBUG --
        enDebugCBT       => FALSE,

        -- MIKUMARI generic --------------------------------------------------------
        enScrambler      => TRUE,
        kHighPrecision   => FALSE,
        -- DEBUG --
        enDebugMikumari  => FALSE
      )
      port map(
        -- System ports -----------------------------------------------------------
        rst           => system_reset,
        pwrOnRst      => pwr_on_reset,
        clkSer        => clk_fast,
        clkPar        => clk_slow,
        clkIndep      => clk_gbe,
        clkIdctrl     => clk_gbe,
        initIn        => power_on_init or cbt_init_from_mutil(i),

        TXP           => MIKUMARI_TXP,
        TXN           => MIKUMARI_TXN,
        RXP           => MIKUMARI_RXP,
        RXN           => MIKUMARI_RXN,
        modClk        => mod_clk,
        tapValueIn    => (others => '0'),
        txBeat        => open,

        -- CBT ports ------------------------------------------------------------
        laneUp        => cbt_lane_up(i),
        idelayErr     => open,
        bitslipErr    => open,
        pattErr       => pattern_error(i),
        watchDogErr   => watchdog_error(i),

        tapValueOut   => tap_value_out(i),
        bitslipNum    => bitslip_num_out(i),
        serdesOffset  => serdes_offset(i),
        firstBitPatt  => open,

        -- Mikumari ports -------------------------------------------------------
        linkUp        => mikumari_link_up(i),

        -- TX port --
        -- Data I/F --
        dataInTx      => miku_data_tx(i),
        validInTx     => miku_valid_tx(i),
        frameLastInTx => miku_last_tx(i),
        txAck         => miku_tx_ack(i),

        pulseIn       => pulse_tx(i),
        pulseTypeTx   => pulse_type_tx(i),
        pulseRegTx    => "0000",
        busyPulseTx   => busy_pulse_tx(i),

        -- RX port --
        -- Data I/F --
        dataOutRx   => miku_data_rx(i),
        validOutRx  => miku_valid_rx(i),
        frameLastRx => miku_last_rx(i),
        checksumErr => checksum_err(i),
        frameBroken => frame_broken(i),
        recvTermnd  => recv_terminated(i),

        pulseOut    => pulse_rx(i),
        pulseTypeRx => pulse_type_rx(i),
        pulseRegRx  => open

      );

    --
    --idelay_tap_in   <= unsigned(tap_value_out);

    u_LACCP : entity mylib.LaccpMainBlock
      generic map
        (
          kPrimaryMode      => false,
          kNumInterconnect  => 1,
          enDebug           => false
        )
      port map
        (
          -- System --------------------------------------------------------
          rst               => laccp_reset(i),
          clk               => clk_slow,

          -- User Interface ------------------------------------------------
          isReadyForDaq     => is_ready_for_daq(i),
          laccpPulsesIn     => (others => '0'),
          laccpPulsesOut    => laccp_pulse_out,
          pulseInRejected   => open,

          -- RLIGP --
          addrMyLink        => sitcp_ip_addr,
          validMyLink       => not emergency_reset,
          addrPartnerLink   => link_addr_partter(i),
          validPartnerLink  => valid_link_addr(i),

          -- RCAP --
          idelayTapIn       => unsigned(tap_value_out(i)),
          serdesLantencyIn  => serdes_offset(i),
          idelayTapOut      => open,
          serdesLantencyOut => open,

          hbuIsSyncedIn     => hbu_is_synchronized,
          syncPulseIn       => '0',
          syncPulseOut      => sync_pulse_out,

          upstreamOffset    => (others => '0'),
          validOffset       => valid_hbc_offset,
          hbcOffset         => hbc_offset,
          fineOffset        => laccp_fine_offset,
          fineOffsetLocal   => local_fine_offset,

          -- LACCP Bus Port ------------------------------------------------
          -- Intra-port--
          isReadyIntraIn    => is_ready_laccp_intra,
          dataIntraIn       => data_laccp_intra_in,
          validIntraIn      => valid_laccp_intra_in,
          dataIntraOut      => data_laccp_intra_out,
          validIntraOut     => valid_laccp_intra_out,

          -- Interconnect --
          isReadyInterIn    => (others => '0'),
          existInterOut     => open,
          dataInterIn       => (others => (others => '0')),
          validInterIn      => (others => '0'),
          dataInterOut      => open,
          validInterOut     => open,

          -- MIKUMARI-Link -------------------------------------------------
          mikuLinkUpIn      => mikumari_link_up(i),

          -- TX port --
          dataTx            => miku_data_tx(i),
          validTx           => miku_valid_tx(i),
          frameLastTx       => miku_last_tx(i),
          txAck             => miku_tx_ack(i),

          pulseTx           => pulse_tx(i),
          pulseTypeTx       => pulse_type_tx(i),
          busyPulseTx       => busy_pulse_tx(i),

          -- RX port --
          dataRx            => miku_data_rx(i),
          validRx           => miku_valid_rx(i),
          frameLastRx       => miku_last_rx(i),
          checkSumErrRx     => checksum_err(i),
          frameBrokenRx     => frame_broken(i),
          recvTermndRx      => recv_terminated(i),

          pulseRx           => pulse_rx(i),
          pulseTypeRx       => pulse_type_rx(i)

        );
  end generate;

  --
  --u_sync_nim1 : entity mylib.synchronizer port map(clk_slow, NIM_IN(1), frame_ctrl_gate);
  frame_ctrl_gate <= '0';
  hbu_reset       <= system_reset when(dip_sw(kStandAlone.Index) = '1') else laccp_reset(0);

  u_HBU : entity mylib.HeartBeatUnit
    generic map
      (
        enDebug           => false
      )
    port map
      (
        -- System --
        rst               => hbu_reset,
        clk               => clk_slow,
        enStandAlone      => DIP(kStandAlone.Index),
        keepLocalHbfNum   => '1',

        -- Sync I/F --
        syncPulseIn       => sync_pulse_out,
        hbcOffsetIn       => hbc_offset,
        validOffsetIn     => valid_hbc_offset,
        isSynchronized    => hbu_is_synchronized,

        -- HeartBeat I/F --
        heartbeatOut      => heartbeat_signal,
        heartbeatCount    => heartbeat_count,
        hbfNumber         => hbf_number,
        hbfNumMismatch    => hbf_num_mismatch,
        clkDiv16          => clk_sys_div16,

        -- DAQ I/F --
        hbfCtrlGateIn     => frame_ctrl_gate,
        forceOn           => '1',
        frameState        => hbf_state,

        hbfFlagsIn        => local_frame_flag,
        frameFlags        => frame_flag_out,

        -- LACCP Bus --
        dataBusIn         => data_laccp_intra_out(GetExtIntraIndex(kPortHBU)),
        validBusIn        => valid_laccp_intra_out(GetExtIntraIndex(kPortHBU)),
        dataBusOut        => data_laccp_intra_in(GetExtIntraIndex(kPortHBU)),
        validBusOut       => valid_laccp_intra_in(GetExtIntraIndex(kPortHBU)),
        isReadyOut        => is_ready_laccp_intra(GetExtIntraIndex(kPortHBU))

      );

  -- MIKUMARI utility ---------------------------------------------------------------------
  u_MUTIL : entity mylib.MikumariUtil
    generic map(
      kNumMikumari => kNumMikumari,
      kSecondaryId => kIdMikuSec
    )
    port map(
      -- System ----------------------------------------------------
      rst               => user_reset,
      clk               => clk_slow,

      clockRootMode     => DIP(kStandAlone.Index),

      -- CBT status ports --
      cbtLaneUp           => cbt_lane_up,
      tapValueIn          => tap_value_out,
      bitslipNumIn        => bitslip_num_out,
      cbtInitOut          => cbt_init_from_mutil,
      tapValueOut         => open,
      rstOverMikuOut      => open,

      -- MIKUMARI Link ports --
      mikuLinkUp          => mikumari_link_up,

      -- LACCP ports --
      laccpUp             => is_ready_for_daq,
      partnerIpAddr       => link_addr_partter,
      hbcOffset           => hbc_offset,
      localFineOffset     => std_logic_vector(local_fine_offset),
      laccpFineOffset     => std_logic_vector(laccp_fine_offset),
      hbfState            => open,

      -- Local bus --
      addrLocalBus        => addr_LocalBus,
      dataLocalBusIn      => data_LocalBusIn,
      dataLocalBusOut     => data_LocalBusOut(kMUTIL.ID),
      reLocalBus          => re_LocalBus(kMUTIL.ID),
      weLocalBus          => we_LocalBus(kMUTIL.ID),
      readyLocalBus       => ready_LocalBus(kMUTIL.ID)
    );

  -- Scaler -------------------------------------------------------------------------------
  scr_en_in(kMsbScr - kIndexRealTime)       <= heartbeat_signal;
  scr_en_in(kMsbScr - kIndexDaqRunTime)     <= heartbeat_signal when(daq_is_runnig = '1') else '0';
  scr_en_in(kMsbScr - kIndexTotalThrotTime) <= scr_thr_on(0);
  scr_en_in(kMsbScr - kIndexInThrot1Time)   <= scr_thr_on(1);
  scr_en_in(kMsbScr - kIndexInThrot2Time)   <= scr_thr_on(2);
  scr_en_in(kMsbScr - kIndexOutThrotTime)   <= scr_thr_on(3);
  scr_en_in(kMsbScr - kIndexHbfThrotTime)   <= scr_thr_on(4);
  scr_en_in(kMsbScr - kIndexMikuError)      <= (pattern_error(kIdMikuSec) or checksum_err(kIdMikuSec) or frame_broken(kIdMikuSec) or recv_terminated(kIdMikuSec)) and is_ready_for_daq(kIdMikuSec);

  scr_en_in(kMsbScr - kIndexTrgReq)         <= '0';
  scr_en_in(kMsbScr - kIndexTrgRejected)    <= '0';

  scr_en_in(kMsbScr - kIndexGate1Time)      <= heartbeat_signal and scr_gate(1);
  scr_en_in(kMsbScr - kIndexGate2Time)      <= heartbeat_signal and scr_gate(2);

  scr_en_in(kNumInput-1 downto 0)           <= swap_vect(hit_out);

  scr_gate(0)   <= '1';
  process(clk_slow)
  begin
    if(clk_slow'event and clk_slow = '1') then
      if(user_reset = '1') then
        scr_gate(kNumScrGate-1 downto 1)  <= (others => '0');
      elsif(heartbeat_signal = '1') then
        scr_gate(1)   <= frame_flag_out(0);
        scr_gate(2)   <= frame_flag_out(1);
      end if;
    end if;
  end process;

  u_SCR: entity mylib.FreeRunScaler
    generic map(
      kNumHitInput        => kNumInput
    )
    port map(
      rst	                => system_reset,
      cntRst              => laccp_pulse_out(kDownPulseCntRst),
      clk	                => clk_slow,

      -- Module Input --
      hbCount             => (heartbeat_count'range => heartbeat_count, others => '0'),
      hbfNum              => (hbf_number'range => hbf_number, others => '0'),
      scrEnIn             => scr_en_in,
      scrRstOut           => open,

      scrgates            => scr_gate,

      -- Local bus --
      addrLocalBus        => addr_LocalBus,
      dataLocalBusIn      => data_LocalBusIn,
      dataLocalBusOut     => data_LocalBusOut(kSCR.ID),
      reLocalBus          => re_LocalBus(kSCR.ID),
      weLocalBus          => we_LocalBus(kSCR.ID),
      readyLocalBus       => ready_LocalBus(kSCR.ID)
      );

  --
  -- Streaming LR-TDC ---------------------------------------------------------------------
  signal_in_merge   <= dcr_d & dcr_u & MAIN_IN_D & MAIN_IN_U;
  strtdc_trigger_in <= laccp_pulse_out(kDownPulseTrigger) or local_trigger_in;

  u_SLT_Inst: entity mylib.StrLrTdc
    generic map(
      kTdcType        => "LRTDC",
      kNumInput       => kNumInput,
      kDivisionRatio  => 8,
      enDEBUG         => false
    )
    port map(
      rst           => user_reset,
      clk           => clk_slow,
      tdcClk        => clk_tdc,

      radiationURE  => uncorrectable_flag,
      daqOn         => daq_is_runnig,
      scrThrEn      => scr_thr_on,
      hitOut        => hit_out,

      -- Data Link -----------------------------------------
      linkActive         => tcp_is_active,

      -- LACCP ------------------------------------------------------
      heartbeatIn       => heartbeat_signal,
      hbCount           => heartbeat_count,
      hbfNumber         => hbf_number,
      ghbfNumMismatchIn => hbf_num_mismatch,
      hbfState          => hbf_state,

      LaccpFineOffset   => laccp_fine_offset,

      frameFlagsIn      => frame_flag_out,

      -- Streaming TDC interface ------------------------------------
      sigIn             => signal_in_merge,
      triggerIn         => strtdc_trigger_in,

      dataRdEn            => vital_rden,
      dataOut             => vital_dout,
      dataRdValid         => vital_valid,

      -- LinkBuffer interface ---------------------------------------
      pfullLinkBufIn      => pfull_link_buf,
      emptyLinkInBufIn    => empty_to_tsd,

      addrLocalBus        => addr_LocalBus,
      dataLocalBusIn      => data_LocalBusIn,
      dataLocalBusOut     => data_LocalBusOut(kTDC.ID),
      reLocalBus          => re_LocalBus(kTDC.ID),
      weLocalBus          => we_LocalBus(kTDC.ID),
      readyLocalBus       => ready_LocalBus(kTDC.ID)
    );

  vital_rden  <= not pfull_link_buf;

  din_link_buf  <= vital_dout(7 downto 0) & vital_dout(15 downto 8) & vital_dout(23 downto 16) & vital_dout(31 downto 24) & vital_dout(39 downto 32) & vital_dout(47 downto 40) & vital_dout(55 downto 48) & vital_dout(63 downto 56);

  u_link_buf : LinkBuffer
    PORT map(
      rst     => user_reset,
      wr_clk  => clk_slow,
      rd_clk  => clk_xgmii,
      din     => din_link_buf,
      wr_en   => vital_valid,
      rd_en   => re_from_tsd,
      dout    => wd_to_tsd,
      full    => open,
      empty   => empty_to_tsd,
      valid   => we_to_tsd,
      prog_full => pfull_link_buf
    );

  -- IOM ------------------------------------------------------------------------
  local_frame_flag(0)   <= dip_sw(kStandAlone.Index) and intsig_from_iom(0);
  local_frame_flag(1)   <= dip_sw(kStandAlone.Index) and intsig_from_iom(1);
  local_trigger_in      <= dip_sw(kStandAlone.Index) and intsig_from_iom(2);

  intsig_to_iom(0)      <= heartbeat_signal;
  intsig_to_iom(1)      <= tcp_is_active;
  intsig_to_iom(2)      <= laccp_pulse_out(kDownPulseTrigger);
  intsig_to_iom(3)      <= frame_flag_out(0);
  intsig_to_iom(4)      <= frame_flag_out(1);
  intsig_to_iom(5)      <= clk_sys_div16;
  intsig_to_iom(6)      <= '1';
  intsig_to_iom(7)      <= '1';


  u_IOM : entity mylib.IOManager
    port map(
      rst	                => user_reset,
      clk	                => clk_slow,

      -- Ext Input
      extInput            => sync_nim_in,
      intOutput           => intsig_from_iom,

      -- Ext Output
      intInput            => intsig_to_iom,
      extOutput           => tmp_nim_out,

      -- Local bus --
      addrLocalBus        => addr_LocalBus,
      dataLocalBusIn      => data_LocalBusIn,
      dataLocalBusOut	    => data_LocalBusOut(kIOM.ID),
      reLocalBus          => re_LocalBus(kIOM.ID),
      weLocalBus          => we_LocalBus(kIOM.ID),
      readyLocalBus	      => ready_LocalBus(kIOM.ID)
      );

  -- C6C -------------------------------------------------------------------------------
  CDCE_PDB  <= '0';
--  u_C6C_Inst : entity mylib.CDCE62002Controller
--    generic map(
--      kSysClkFreq         => 125_000_000
--      )
--    port map(
--      rst                 => system_reset,
--      clk                 => clk_slow,
--      refClkIn            => gmod_clk,
--
--      chipReset           => c6c_reset,
--      clkIndep            => clk_sys,
--      chipLock            => CDCE_LOCK,
--
--      -- Module output --
--      PDB                 => CDCE_PDB,
--      REF_CLKP            => CDCE_REFP,
--      REF_CLKN            => CDCE_REFN,
--      CSB_SPI             => CDCE_LE,
--      SCLK_SPI            => CDCE_SCLK,
--      MOSI_SPI            => CDCE_SI,
--      MISO_SPI            => CDCE_SO,
--
--      -- Local bus --
--      addrLocalBus        => addr_LocalBus,
--      dataLocalBusIn      => data_LocalBusIn,
--      dataLocalBusOut     => data_LocalBusOut(kC6C.ID),
--      reLocalBus          => re_LocalBus(kC6C.ID),
--      weLocalBus          => we_LocalBus(kC6C.ID),
--      readyLocalBus       => ready_LocalBus(kC6C.ID)
--    );

  -- MIG -------------------------------------------------------------------------------

  -- TSD -------------------------------------------------------------------------------
  u_TSD_Inst : entity mylib.TCPsenderXG
    port map(
      RST                     => pwr_on_reset,
      CLK                     => clk_xgmii,

      -- data from EVB --
      rdFromEVB               => wd_to_tsd,
      rvFromEVB               => we_to_tsd,
      emptyFromEVB            => empty_to_tsd,
      reToEVB                 => re_from_tsd,

      -- data to SiTCP
      isActive                => tcp_is_active,
      afullTx                 => tcp_tx_afull,
      tcpTxB	                => tcp_txb,
      tcpTxD	                => tcp_txd
      );

  -- SDS --------------------------------------------------------------------
  u_SDS_Inst : entity mylib.SelfDiagnosisSystem
    port map(
      rst               => user_reset,
      clk               => clk_slow,
      clkIcap           => clk_spi,

      -- Module input  --
      VP                => VP,
      VN                => VN,

      -- Module output --
      shutdownOverTemp  => shutdown_over_temp,
      uncorrectableAlarm => uncorrectable_flag,

      -- Local bus --
      addrLocalBus      => addr_LocalBus,
      dataLocalBusIn    => data_LocalBusIn,
      dataLocalBusOut   => data_LocalBusOut(kSDS.ID),
      reLocalBus        => re_LocalBus(kSDS.ID),
      weLocalBus        => we_LocalBus(kSDS.ID),
      readyLocalBus     => ready_LocalBus(kSDS.ID)
      );


  -- FMP --------------------------------------------------------------------
  u_FMP_Inst : entity mylib.FlashMemoryProgrammer
    port map(
      rst               => user_reset,
      clk               => clk_slow,
      clkSpi            => clk_spi,

      -- Module output --
      CS_SPI            => FCSB,
--      SCLK_SPI          => USR_CLK,
      MOSI_SPI          => MOSI,
      MISO_SPI          => DIN,

      -- Local bus --
      addrLocalBus      => addr_LocalBus,
      dataLocalBusIn    => data_LocalBusIn,
      dataLocalBusOut   => data_LocalBusOut(kFMP.ID),
      reLocalBus        => re_LocalBus(kFMP.ID),
      weLocalBus        => we_LocalBus(kFMP.ID),
      readyLocalBus     => ready_LocalBus(kFMP.ID)
      );


  -- BCT -------------------------------------------------------------------------------
  -- Actual local bus
  u_BCT_Inst : entity mylib.BusController
    port map(
      rstSys                    => bct_reset,
      rstFromBus                => rst_from_bus,
      reConfig                  => PROGB_ON,
      clk                       => clk_slow,
      -- Local Bus --
      addrLocalBus              => addr_LocalBus,
      dataFromUserModules       => data_LocalBusOut,
      dataToUserModules         => data_LocalBusIn,
      reLocalBus                => re_LocalBus,
      weLocalBus                => we_LocalBus,
      readyLocalBus             => ready_LocalBus,
      -- RBCP --
      addrRBCP                  => rbcp_addr,
      wdRBCP                    => rbcp_wd,
      weRBCP                    => rbcp_we,
      reRBCP                    => rbcp_re,
      ackRBCP                   => rbcp_ack,
      rdRBCP                    => rbcp_rd
      );

  -- SiTCP Inst ------------------------------------------------------------------------
  u_SyncRstMiku : entity mylib.synchronizer port map(clk_xgmii, rst_from_miku or (not pcs_pma_status(kPcsBlockLock)), rst_from_miku_xgmii);

  u_SiTCPXG_Inst : WRAP_SiTCPXG_XC7K_128K
    generic map(
      RxBufferSize	=> "LongLong"
      )
    port map(
      REG_FPGA_VER              => kCurrentVersion,
      REG_FPGA_ID               => X"00000000",

      --		==== System I/F ====
      FORCE_DEFAULTn            => dip_sw(kSiTCP.Index),
      XGMII_CLOCK               => clk_xgmii,
      RSTs                      => system_reset_xgmii or rst_from_miku_xgmii,

      --		==== XGMII I/F ====
      XGMII_RXC 								=> xgmii_rxc,
      XGMII_RXD 								=> xgmii_rxd,
      XGMII_TXC									=> xgmii_txc,
			XGMII_TXD									=> xgmii_txd,

      --		==== 93C46 I/F ====
			EEPROM_CS									=> EEP_CS(1),
			EEPROM_SK									=> EEP_SK(1),
			EEPROM_DI									=> EEP_DI(1),
      EEPROM_DO									=> EEP_DO(1),

      --		==== User I/F ====
      SiTCP_RESET_OUT	          => emergency_reset,
      IP_ADDR                   => sitcp_ip_addr,
      --		--- RBCP ---
			RBCP_ACT                  => open,
			RBCP_ADDR                 => rbcp_xg_addr,
			RBCP_WE										=> rbcp_xg_we,
			RBCP_WD										=> rbcp_xg_wd,
			RBCP_RE										=> rbcp_xg_re,
      RBCP_ACK                  => rbcp_xg_ack,
			RBCP_RD	                  => rbcp_xg_rd,
      -- --- TCP ---
      USER_SESSION_OPEN_REQ	    => '0',
		 	USER_SESSION_ESTABLISHED  => tcp_is_active,
		 	USER_SESSION_CLOSE_REQ    => close_req,
      USER_SESSION_CLOSE_ACK    => close_ack,
      USER_TX_D                 => tcp_txd,
      USER_TX_B	                => tcp_txb,
		 	USER_TX_AFULL             => tcp_tx_afull,
      USER_RX_SIZE              => X"FFF0",
		 	USER_RX_CLR_ENB	          => tcp_rx_clr_enb,
      USER_RX_CLR_REQ           => tcp_rx_clr_enb,
      USER_RX_RADR 							=> tcp_rx_wadr,
      USER_RX_WADR 							=> tcp_rx_wadr,
      USER_RX_WENB 							=> open,
      USER_RX_WDAT 							=> open
	);


  --aaa
  u_RbcpCdc : entity mylib.RbcpCdc
    port map(
      -- Mikumari clock domain --
      rstSys      => system_reset,
      clkSys      => clk_slow,
      rbcpAddr    => rbcp_addr,
      rbcpWd      => rbcp_wd,
      rbcpWe      => rbcp_we,
      rbcpRe      => rbcp_re,
      rbcpAck     => rbcp_ack,
      rbcpRd      => rbcp_rd,

      -- GMII clock domain --
      rstXgmii    => system_reset_xgmii,
      clkXgmii    => clk_xgmii,
      rbcpXgAddr  => rbcp_xg_addr,
      rbcpXgWd    => rbcp_xg_wd,
      rbcpXgWe    => rbcp_xg_we,
      rbcpXgRe    => rbcp_xg_re,
      rbcpXgAck   => rbcp_xg_ack,
      rbcpXgRd    => rbcp_xg_rd
      );

  u_gTCP_inst : entity mylib.global_sitcp_manager
    port map(
      RST           => system_reset_xgmii,
      CLK           => clk_xgmii,
      ACTIVE        => tcp_is_active,
      REQ           => close_req,
      ACT           => close_ack,
      rstFromTCP    => open
      );



  -- SFP transceiver -------------------------------------------------------------------
  u_PCSPMA_Inst : ten_gig_eth_pcs_pma
  port map(
    -- Core block --
    refclk_p              => GTX_REFCLK_P,
    refclk_n              => GTX_REFCLK_N,
    reset                 => core_master_reset,
    resetdone_out         => open,
    coreclk_out           => clk_xgmii,

    -- Tranceiver --
    txp 									=> GTX_TX_P,
    txn 									=> GTX_TX_N,
    rxp 									=> GTX_RX_P,
    rxn 									=> GTX_RX_N,

    -- SFP+ IF --
    signal_detect         => '1',
    tx_disable            => open,
    tx_fault              => '0',

    -- XGMII --
    xgmii_txd             => xgmii_txd,
    xgmii_txc             => xgmii_txc,
    xgmii_rxd             => xgmii_rxd,
    xgmii_rxc             => xgmii_rxc,

    -- MDIO --
    mdc                   => '1',
    mdio_in               => '1',
    mdio_out              => open,
    mdio_tri              => open,
    prtad                 => "00000",

    -- DRP IF --
    dclk                  => clk_xgmii,
    drp_req               => drp_req,
    drp_gnt               => drp_req,

    drp_den_o             => drp_den,
    drp_dwe_o             => drp_dwe,
    drp_daddr_o           => drp_addr,
    drp_di_o              => drp_di,
    drp_drdy_o            => drp_drdy,
    drp_drpdo_o           => drp_drpdo,
    drp_den_i             => drp_den,
    drp_dwe_i             => drp_dwe,
    drp_daddr_i           => drp_addr,
    drp_di_i              => drp_di,
    drp_drdy_i            => drp_drdy,
    drp_drpdo_i           => drp_drpdo,

    -- mics --
    pma_pmd_type          => "111",
    core_status           => pcs_pma_status,
    sim_speedup_control   => '0',

    txusrclk_out          => open,
    txusrclk2_out         => open,
    rxrecclk_out          => open,
    areset_datapathclk_out => open,
    gttxreset_out         => open,
    gtrxreset_out         => open,
    txuserrdy_out         => open,
    reset_counter_done_out => open,
    qplllock_out          => qpll_locked,
    qplloutclk_out        => open,
    qplloutrefclk_out     => open
    );


  -- Clock inst ------------------------------------------------------------------------
  --clk_slow  <= clk_sys;
  u_ClkMan_Inst   : clk_wiz_sys
    port map (
      -- Clock out ports
      clk_sys         => clk_sys,
      clk_indep_gtx   => clk_gbe,
      clk_spi         => clk_spi,
      -- Status and control signals
      reset           => '0',
      locked          => clk_sys_locked,
      -- Clock in ports
      clk_in1_p       => BASE_CLKP,
      clk_in1_n       => BASE_CLKN
      );

  u_BUFG :  BUFG
    port map (
      O => gmod_clk, -- 1-bit output: Clock output
      I => mod_clk  -- 1-bit input: Clock input
    );
--
--  --
--  u_BUFG_Fast_inst :  BUFG
--    port map (
--      O => clk_fast, -- 1-bit output: Clock output
--      I => c6c_fast  -- 1-bit input: Clock input
--    );
--
--  u_BUFG_Slow_inst :  BUFG
--    port map (
--      O => clk_slow, -- 1-bit output: Clock output
--      I => c6c_slow  -- 1-bit input: Clock input
--    );

  --
  u_CdcmMan_Inst : mmcm_cdcm
  port map(
    -- Clock in ports
--        clkfb_in          => clk_gfb,
      -- Clock out ports
      clk_slow        => clk_slow,
      clk_fast        => clk_fast,
      clk_tdc0        => clk_tdc(0),
      clk_tdc1        => clk_tdc(1),
      clk_tdc2        => clk_tdc(2),
      clk_tdc3        => clk_tdc(3),
--        clkfb_out         => clk_fb,
      -- Status and control signals
      reset           => mmcm_cdcm_reset,
      locked          => mmcm_cdcm_locked,
      clk_in1         => clk_sys,
      clk_in2         => gmod_clk,
      clk_in_sel      => dip_sw(kStandAlone.Index)
      );


--  u_IBUFDS_SLOW_inst : IBUFDS
--    generic map (
--       DIFF_TERM => FALSE, -- Differential Termination
--       IBUF_LOW_PWR => FALSE, -- Low power (TRUE) vs. performance (FALSE) setting for referenced I/O standards
--       IOSTANDARD => "LVDS")
--    port map (
--       O => c6c_slow,  -- Buffer output
--       I => CLK_SLOWP,  -- Diff_p buffer input (connect directly to top-level port)
--       IB => CLK_SLOWN -- Diff_n buffer input (connect directly to top-level port)
--       );
--
--  u_IBUFDS_FAST_inst : IBUFDS
--    generic map (
--       DIFF_TERM => FALSE, -- Differential Termination
--       IBUF_LOW_PWR => FALSE, -- Low power (TRUE) vs. performance (FALSE) setting for referenced I/O standards
--       IOSTANDARD => "LVDS")
--    port map (
--       O => c6c_fast,  -- Buffer output
--       I => CLK_FASTP,  -- Diff_p buffer input (connect directly to top-level port)
--       IB => CLK_FASTN -- Diff_n buffer input (connect directly to top-level port)
--       );
--
end Behavioral;
