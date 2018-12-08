// Copyright (c) 2012-2013 Ludvig Strigeus
// This program is GPL Licensed. See COPYING for the full license.

`timescale 1ns / 1ps

// Asynchronous PSRAM controller for byte access
// After outputting a byte to read, the result is available 70ns later.
module MemoryController(
                 input clk,
                 input read_a,             // Set to 1 to read from RAM
                 input read_b,             // Set to 1 to read from RAM
                 input write,              // Set to 1 to write to RAM
                 input [23:0] addr,        // Address to read / write
                 input [7:0] din,          // Data to write
                 output reg [7:0] dout_a,  // Last read data a
                 output reg [7:0] dout_b,  // Last read data b
                 output reg busy,          // 1 while an operation is in progress

                 output reg MemOE,         // Output Enable. Enable when Low.
                 output reg MemWR,         // Write Enable. WRITE when Low.
                 output MemAdv,            // Address valid. Keep LOW for Async.
                 output MemClk,            // Clock for sync oper. Keep LOW for Async.
                 output reg RamCS,         // Chip Enable. Active = LOW
                 output RamCRE,            // CRE = Control Register. LOW = Normal mode.
                 output reg RamUB,         // Upper byte enable
                 output reg RamLB,         // Lower byte enable
                 output reg [22:0] MemAdr,
                 inout [15:0] MemDB);
                 
  // These are always low for async operations
  assign MemAdv = 0;
  assign MemClk = 0;
  assign RamCRE = 0;
  reg [7:0] data_to_write;
  assign MemDB = MemOE ? {data_to_write, data_to_write} : 16'bz; // MemOE == 0 means we need to output tristate
  reg [1:0] cycles;
  reg r_read_a;
  
  always @(posedge clk) begin
    // Initiate read or write
    if (!busy) begin
      if (read_a || read_b || write) begin
        MemAdr <= addr[23:1];
        RamUB <= !(addr[0] != 0); // addr[0] == 0 asserts RamUB, active low.
        RamLB <= !(addr[0] == 0);
        RamCS <= 0;    // 0 means active
        MemWR <= !(write != 0); // Active Low
        MemOE <= !(write == 0);
        busy <= 1;
        data_to_write <= din;
        cycles <= 0;
        r_read_a <= read_a;
      end else begin
        MemOE <= 1;
        MemWR <= 1;
        RamCS <= 1;
        RamUB <= 1;
        RamLB <= 1;
        busy <= 0;
        cycles <= 0;
      end
    end else begin
      if (cycles == 2) begin
        // Now we have waited 3x45 = 135ns, latch incoming data on read.
        if (!MemOE) begin
          if (r_read_a) dout_a <= RamUB ? MemDB[7:0] : MemDB[15:8];
          else dout_b <= RamUB ? MemDB[7:0] : MemDB[15:8];
        end
        MemOE <= 1; // Deassert Output Enable.
        MemWR <= 1; // Deassert Write
        RamCS <= 1; // Deassert Chip Select
        RamUB <= 1; // Deassert upper/lower byte
        RamLB <= 1;
        busy <= 0;
        cycles <= 0;
      end else begin
        cycles <= cycles + 1;
      end
    end
  end
endmodule  // MemoryController

// Module reads bytes and writes to proper address in ram.
// Done is asserted when the whole game is loaded.
// This parses iNES headers too.
module GameLoader(input clk, input reset,
                  input [7:0] indata, input indata_clk,
                  output reg [21:0] mem_addr, output [7:0] mem_data, output mem_write,
                  output [31:0] mapper_flags,
                  output reg done,
                  output error);
  reg [1:0] state = 0;
  reg [7:0] prgsize;
  reg [3:0] ctr;
  reg [7:0] ines[0:15]; // 16 bytes of iNES header
  reg [21:0] bytes_left;
  
  assign error = (state == 3);
  wire [7:0] prgrom = ines[4];
  wire [7:0] chrrom = ines[5];
  assign mem_data = indata;
  assign mem_write = (bytes_left != 0) && (state == 1 || state == 2) && indata_clk;
  
  wire [2:0] prg_size = prgrom <= 1  ? 0 :
                        prgrom <= 2  ? 1 : 
                        prgrom <= 4  ? 2 : 
                        prgrom <= 8  ? 3 : 
                        prgrom <= 16 ? 4 : 
                        prgrom <= 32 ? 5 : 
                        prgrom <= 64 ? 6 : 7;
                        
  wire [2:0] chr_size = chrrom <= 1  ? 0 : 
                        chrrom <= 2  ? 1 : 
                        chrrom <= 4  ? 2 : 
                        chrrom <= 8  ? 3 : 
                        chrrom <= 16 ? 4 : 
                        chrrom <= 32 ? 5 : 
                        chrrom <= 64 ? 6 : 7;
  
  wire [7:0] mapper = {ines[7][7:4], ines[6][7:4]};
  wire has_chr_ram = (chrrom == 0);
  assign mapper_flags = {16'b0, has_chr_ram, ines[6][0], chr_size, prg_size, mapper};
  always @(posedge clk) begin
    if (reset) begin
      state <= 0;
      done <= 0;
      ctr <= 0;
      mem_addr <= 0;  // Address for PRG
    end else begin
      case(state)
      // Read 16 bytes of ines header
      0: if (indata_clk) begin
           ctr <= ctr + 1;
           ines[ctr] <= indata;
           bytes_left <= {prgrom, 14'b0};
           if (ctr == 4'b1111)
             state <= (ines[0] == 8'h4E) && (ines[1] == 8'h45) && (ines[2] == 8'h53) && (ines[3] == 8'h1A) && !ines[6][2] && !ines[6][3] ? 1 : 3;
         end
      1, 2: begin // Read the next |bytes_left| bytes into |mem_addr|
          if (bytes_left != 0) begin
            if (indata_clk) begin
              bytes_left <= bytes_left - 1;
              mem_addr <= mem_addr + 1;
            end
          end else if (state == 1) begin
            state <= 2;
            mem_addr <= 22'b10_0000_0000_0000_0000_0000; // Address for CHR
            bytes_left <= {1'b0, chrrom, 13'b0};
          end else if (state == 2) begin
            done <= 1;
          end
        end
      endcase
    end
  end
endmodule


module NES_Nexys4(input CLK100MHZ,
                 input CPU_RESET,
                 input [4:0] BTN,
                 input [15:0] SW,
                 output [15:0] LED,
                 output [7:0] SSEG_CA,
                 output [7:0] SSEG_AN,
                 // UART
                 input UART_RXD,
                 output UART_TXD,
                 // VGA
                 output vga_v, output vga_h, output [3:0] vga_r, output [3:0] vga_g, output [3:0] vga_b,
                 // Memory
                 output MemOE,          // Output Enable. Enable when Low.
                 output MemWR,          // Write Enable. WRITE when Low.
                 output MemAdv,         // Address valid. Keep LOW for Async.
                 input MemWait,         // Ignore for Async.
                 output MemClk,         // Clock for sync oper. Keep LOW for Async.
                 output RamCS,          // Chip Enable. Active = LOW
                 output RamCRE,         // CRE = Control Register. LOW = Normal mode.
                 output RamUB,          // Upper byte enable
                 output RamLB,          // Lower byte enable
                 output [22:0] MemAdr,
                 inout [15:0] MemDB,
                 // Sound board
                 output AUD_MCLK,
                 output AUD_LRCK,
                 output AUD_SCK,
                 output AUD_SDIN
                 );

  wire clock_locked;
  wire clk;
  clk_wiz_v3_6 clock_21mhz(.CLK_IN1(CLK100MHZ), .CLK_OUT1(clk),  .RESET(1'b0), .LOCKED(clock_locked));

  // UART
  wire [7:0] uart_data;
  wire [7:0] uart_addr;
  wire       uart_write;
  wire       uart_error;
  UartDemux uart_demux(clk, 1'b0, UART_RXD, uart_data, uart_addr, uart_write, uart_error);
  assign     UART_TXD = 1;

  // Loader
  wire [7:0] loader_input = uart_data;
  wire       loader_clk   = (uart_addr == 8'h37) && uart_write;
  reg  [7:0] loader_conf;
  reg  [7:0] loader_btn, loader_btn_2;
  always @(posedge clk) begin
    if (uart_addr == 8'h35 && uart_write)
      loader_conf <= uart_data;
    if (uart_addr == 8'h40 && uart_write)
      loader_btn <= uart_data;
    if (uart_addr == 8'h41 && uart_write)
      loader_btn_2 <= uart_data;
  end

  // NES Palette -> RGB332 conversion
  reg [14:0] pallut[0:63];
  initial $readmemh("nes_palette.txt", pallut);

  // LED Display
  reg [31:0] led_value;
  reg [7:0] led_enable;
  LedDriver led_driver(clk, led_value, led_enable, SSEG_CA, SSEG_AN);

  wire [8:0] cycle;
  wire [8:0] scanline;
  wire [15:0] sample;
  wire [5:0] color;
  wire joypad_strobe;
  wire [1:0] joypad_clock;
  wire [21:0] memory_addr;
  wire memory_read_cpu, memory_read_ppu;
  wire memory_write;
  wire [7:0] memory_din_cpu, memory_din_ppu;
  wire [7:0] memory_dout;
  reg [7:0] joypad_bits, joypad_bits2;
  reg [1:0] last_joypad_clock;
  wire [31:0] dbgadr;
  wire [1:0] dbgctr;

  reg [1:0] nes_ce;

  always @(posedge clk) begin
    if (joypad_strobe) begin
      joypad_bits <= loader_btn;
      joypad_bits2 <= loader_btn_2;
    end
    if (!joypad_clock[0] && last_joypad_clock[0])
      joypad_bits <= {1'b0, joypad_bits[7:1]};
    if (!joypad_clock[1] && last_joypad_clock[1])
      joypad_bits2 <= {1'b0, joypad_bits2[7:1]};
    last_joypad_clock <= joypad_clock;
  end
  
  wire [21:0] loader_addr;
  wire [7:0] loader_write_data;
  wire loader_reset = loader_conf[0];
  wire loader_write;
  wire [31:0] mapper_flags;
  wire loader_done, loader_fail;
  GameLoader loader(clk, loader_reset, loader_input, loader_clk,
                    loader_addr, loader_write_data, loader_write,
                    mapper_flags, loader_done, loader_fail);

  wire reset_nes = (BTN[3] || !loader_done);
  wire run_mem = (nes_ce == 0) && !reset_nes;
  wire run_nes = (nes_ce == 3) && !reset_nes;

  // NES is clocked at every 4th cycle.
  always @(posedge clk)
    nes_ce <= nes_ce + 1;
    
  NES nes(clk, reset_nes, run_nes,
          mapper_flags,
          sample, color,
          joypad_strobe, joypad_clock, {joypad_bits2[0], joypad_bits[0]},
          SW[4:0],
          memory_addr,
          memory_read_cpu, memory_din_cpu,
          memory_read_ppu, memory_din_ppu,
          memory_write, memory_dout,
          cycle, scanline,
          dbgadr,
          dbgctr);

  // This is the memory controller to access the board's PSRAM
  wire ram_busy;
  MemoryController memory(clk,
                          memory_read_cpu && run_mem, 
                          memory_read_ppu && run_mem,
                          memory_write && run_mem || loader_write,
                          loader_write ? {2'b00, loader_addr} : {2'b00, memory_addr},
                          loader_write ? loader_write_data : memory_dout,
                          memory_din_cpu,
                          memory_din_ppu,
                          ram_busy,
                          MemOE, MemWR, MemAdv, MemClk, RamCS, RamCRE, RamUB, RamLB, MemAdr, MemDB);
  reg ramfail;
  always @(posedge clk) begin
    if (loader_reset)
      ramfail <= 0;
    else
      ramfail <= ram_busy && loader_write || ramfail;
  end

  wire [14:0] doubler_pixel;
  wire doubler_sync;
  wire [9:0] vga_hcounter, doubler_x;
  wire [9:0] vga_vcounter;
  
  VgaDriver vga(clk, vga_h, vga_v, vga_r, vga_g, vga_b, vga_hcounter, vga_vcounter, doubler_x, doubler_pixel, doubler_sync, SW[0]);
  
  wire [14:0] pixel_in = pallut[color];
  Hq2x hq2x(clk, pixel_in, SW[5], 
            scanline[8],        // reset_frame
            (cycle[8:3] == 42), // reset_line
            doubler_x,          // 0-511 for line 1, or 512-1023 for line 2.
            doubler_sync,       // new frame has just started
            doubler_pixel);     // pixel is outputted

  wire [15:0] sound_signal = {sample[15] ^ 1'b1, sample[14:0]};
//  wire [15:0] sound_signal_fir;
//  wire sample_now_fir;
//  FirFilter fir(clk, sound_signal, sound_signal_fir, sample_now_fir);
  // Display mapper info on screen
  always @(posedge clk) begin
    led_enable <= 255;
    led_value <= sound_signal;
  end

  reg [7:0] sound_ctr;
  always @(posedge clk)
    sound_ctr <= sound_ctr + 1;
  wire sound_load = /*SW[6] ? sample_now_fir : */(sound_ctr == 0);
  SoundDriver sound_driver(clk, 
      /*SW[6] ? sound_signal_fir : */sound_signal, 
      sound_load,
      sound_load,
      AUD_MCLK, AUD_LRCK, AUD_SCK, AUD_SDIN);
 
  assign LED = {12'b0, uart_error, ramfail, loader_fail, loader_done};
endmodule

module MicroCodeTableInner(input clk, input ce, input reset, input [7:0] IR, input [2:0] State, output reg [8:0] M);
  reg [8:0] L[0:2047];
  initial begin
L[0] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2] = 9'b00_10_10100; // ['PCH->[SP--]', 'PCL->[SP--],', 'PCH->[SP--](KEEPAC)', 'PCL->[SP--](KEEPAC)']
L[3] = 9'b00_10_10100; // ['PCH->[SP--]', 'PCL->[SP--],', 'PCH->[SP--](KEEPAC)', 'PCL->[SP--](KEEPAC)']
L[4] = 9'b00_10_10101; // P->[SP--]
L[5] = 9'b00_11_00011; // [VECT]->T
L[6] = 9'b10_11_10011; // [VECT]:T->PC
L[7] = 9'bxx_xx_xxxxx; // []
L[8] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[9] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[10] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[11] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[12] = 9'b00_01_01011; // [AX]->AH,T->AL
L[13] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[14] = 9'bxx_xx_xxxxx; // []
L[15] = 9'bxx_xx_xxxxx; // []
L[16] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[17] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[18] = 9'bxx_xx_xxxxx; // []
L[19] = 9'bxx_xx_xxxxx; // []
L[20] = 9'bxx_xx_xxxxx; // []
L[21] = 9'bxx_xx_xxxxx; // []
L[22] = 9'bxx_xx_xxxxx; // []
L[23] = 9'bxx_xx_xxxxx; // []
L[24] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[25] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[26] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[27] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[28] = 9'b00_01_01011; // [AX]->AH,T->AL
L[29] = 9'b00_01_00011; // [AX]->T
L[30] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[31] = 9'b10_01_00101; // T->[AX]
L[32] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[33] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[34] = 9'bxx_xx_xxxxx; // []
L[35] = 9'bxx_xx_xxxxx; // []
L[36] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[37] = 9'bxx_xx_xxxxx; // []
L[38] = 9'bxx_xx_xxxxx; // []
L[39] = 9'bxx_xx_xxxxx; // []
L[40] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[41] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[42] = 9'bxx_xx_xxxxx; // []
L[43] = 9'bxx_xx_xxxxx; // []
L[44] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[45] = 9'bxx_xx_xxxxx; // []
L[46] = 9'bxx_xx_xxxxx; // []
L[47] = 9'bxx_xx_xxxxx; // []
L[48] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[49] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[50] = 9'bxx_xx_xxxxx; // []
L[51] = 9'bxx_xx_xxxxx; // []
L[52] = 9'b00_01_00011; // [AX]->T
L[53] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[54] = 9'b10_01_00101; // T->[AX]
L[55] = 9'bxx_xx_xxxxx; // []
L[56] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[57] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[58] = 9'bxx_xx_xxxxx; // []
L[59] = 9'bxx_xx_xxxxx; // []
L[60] = 9'b00_01_00011; // [AX]->T
L[61] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[62] = 9'b10_01_00101; // T->[AX]
L[63] = 9'bxx_xx_xxxxx; // []
L[64] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[65] = 9'b00_00_00011; // [PC]->
L[66] = 9'b10_10_01111; // P->[SP--]
L[67] = 9'bxx_xx_xxxxx; // []
L[68] = 9'bxx_xx_xxxxx; // []
L[69] = 9'bxx_xx_xxxxx; // []
L[70] = 9'bxx_xx_xxxxx; // []
L[71] = 9'bxx_xx_xxxxx; // []
L[72] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[73] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[74] = 9'bxx_xx_xxxxx; // []
L[75] = 9'bxx_xx_xxxxx; // []
L[76] = 9'bxx_xx_xxxxx; // []
L[77] = 9'bxx_xx_xxxxx; // []
L[78] = 9'bxx_xx_xxxxx; // []
L[79] = 9'bxx_xx_xxxxx; // []
L[80] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[81] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[82] = 9'bxx_xx_xxxxx; // []
L[83] = 9'bxx_xx_xxxxx; // []
L[84] = 9'bxx_xx_xxxxx; // []
L[85] = 9'bxx_xx_xxxxx; // []
L[86] = 9'bxx_xx_xxxxx; // []
L[87] = 9'bxx_xx_xxxxx; // []
L[88] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[89] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[90] = 9'bxx_xx_xxxxx; // []
L[91] = 9'bxx_xx_xxxxx; // []
L[92] = 9'bxx_xx_xxxxx; // []
L[93] = 9'bxx_xx_xxxxx; // []
L[94] = 9'bxx_xx_xxxxx; // []
L[95] = 9'bxx_xx_xxxxx; // []
L[96] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[97] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[98] = 9'b11_00_00001; // [PC++]->AH
L[99] = 9'bxx_xx_xxxxx; // []
L[100] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[101] = 9'bxx_xx_xxxxx; // []
L[102] = 9'bxx_xx_xxxxx; // []
L[103] = 9'bxx_xx_xxxxx; // []
L[104] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[105] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[106] = 9'b11_00_00001; // [PC++]->AH
L[107] = 9'bxx_xx_xxxxx; // []
L[108] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[109] = 9'bxx_xx_xxxxx; // []
L[110] = 9'bxx_xx_xxxxx; // []
L[111] = 9'bxx_xx_xxxxx; // []
L[112] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[113] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[114] = 9'b11_00_00001; // [PC++]->AH
L[115] = 9'bxx_xx_xxxxx; // []
L[116] = 9'b00_01_00011; // [AX]->T
L[117] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[118] = 9'b10_01_00101; // T->[AX]
L[119] = 9'bxx_xx_xxxxx; // []
L[120] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[121] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[122] = 9'b11_00_00001; // [PC++]->AH
L[123] = 9'bxx_xx_xxxxx; // []
L[124] = 9'b00_01_00011; // [AX]->T
L[125] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[126] = 9'b10_01_00101; // T->[AX]
L[127] = 9'bxx_xx_xxxxx; // []
L[128] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[129] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[130] = 9'b11_00_10001; // PC+T->PC
L[131] = 9'bxx_xx_xxxxx; // []
L[132] = 9'b10_00_00011; // ['NO-OP', '']
L[133] = 9'bxx_xx_xxxxx; // []
L[134] = 9'bxx_xx_xxxxx; // []
L[135] = 9'bxx_xx_xxxxx; // []
L[136] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[137] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[138] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[139] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[140] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[141] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[142] = 9'bxx_xx_xxxxx; // []
L[143] = 9'bxx_xx_xxxxx; // []
L[144] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[145] = 9'b10_00_00011; // ['NO-OP', '']
L[146] = 9'bxx_xx_xxxxx; // []
L[147] = 9'bxx_xx_xxxxx; // []
L[148] = 9'bxx_xx_xxxxx; // []
L[149] = 9'bxx_xx_xxxxx; // []
L[150] = 9'bxx_xx_xxxxx; // []
L[151] = 9'bxx_xx_xxxxx; // []
L[152] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[153] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[154] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[155] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[156] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[157] = 9'b00_01_00011; // [AX]->T
L[158] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[159] = 9'b10_01_00101; // T->[AX]
L[160] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[161] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[162] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[163] = 9'bxx_xx_xxxxx; // []
L[164] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[165] = 9'bxx_xx_xxxxx; // []
L[166] = 9'bxx_xx_xxxxx; // []
L[167] = 9'bxx_xx_xxxxx; // []
L[168] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[169] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[170] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[171] = 9'bxx_xx_xxxxx; // []
L[172] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[173] = 9'bxx_xx_xxxxx; // []
L[174] = 9'bxx_xx_xxxxx; // []
L[175] = 9'bxx_xx_xxxxx; // []
L[176] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[177] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[178] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[179] = 9'bxx_xx_xxxxx; // []
L[180] = 9'b00_01_00011; // [AX]->T
L[181] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[182] = 9'b10_01_00101; // T->[AX]
L[183] = 9'bxx_xx_xxxxx; // []
L[184] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[185] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[186] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[187] = 9'bxx_xx_xxxxx; // []
L[188] = 9'b00_01_00011; // [AX]->T
L[189] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[190] = 9'b10_01_00101; // T->[AX]
L[191] = 9'bxx_xx_xxxxx; // []
L[192] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[193] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[194] = 9'bxx_xx_xxxxx; // []
L[195] = 9'bxx_xx_xxxxx; // []
L[196] = 9'bxx_xx_xxxxx; // []
L[197] = 9'bxx_xx_xxxxx; // []
L[198] = 9'bxx_xx_xxxxx; // []
L[199] = 9'bxx_xx_xxxxx; // []
L[200] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[201] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[202] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[203] = 9'bxx_xx_xxxxx; // []
L[204] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[205] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[206] = 9'bxx_xx_xxxxx; // []
L[207] = 9'bxx_xx_xxxxx; // []
L[208] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[209] = 9'b10_00_00011; // ['NO-OP', '']
L[210] = 9'bxx_xx_xxxxx; // []
L[211] = 9'bxx_xx_xxxxx; // []
L[212] = 9'bxx_xx_xxxxx; // []
L[213] = 9'bxx_xx_xxxxx; // []
L[214] = 9'bxx_xx_xxxxx; // []
L[215] = 9'bxx_xx_xxxxx; // []
L[216] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[217] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[218] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[219] = 9'bxx_xx_xxxxx; // []
L[220] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[221] = 9'b00_01_00011; // [AX]->T
L[222] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[223] = 9'b10_01_00101; // T->[AX]
L[224] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[225] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[226] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[227] = 9'bxx_xx_xxxxx; // []
L[228] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[229] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[230] = 9'bxx_xx_xxxxx; // []
L[231] = 9'bxx_xx_xxxxx; // []
L[232] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[233] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[234] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[235] = 9'bxx_xx_xxxxx; // []
L[236] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[237] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[238] = 9'bxx_xx_xxxxx; // []
L[239] = 9'bxx_xx_xxxxx; // []
L[240] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[241] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[242] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[243] = 9'bxx_xx_xxxxx; // []
L[244] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[245] = 9'b00_01_00011; // [AX]->T
L[246] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[247] = 9'b10_01_00101; // T->[AX]
L[248] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[249] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[250] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[251] = 9'bxx_xx_xxxxx; // []
L[252] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[253] = 9'b00_01_00011; // [AX]->T
L[254] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[255] = 9'b10_01_00101; // T->[AX]
L[256] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[257] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[258] = 9'b00_10_10100; // ['PCH->[SP--]', 'PCL->[SP--],', 'PCH->[SP--](KEEPAC)', 'PCL->[SP--](KEEPAC)']
L[259] = 9'b00_10_10100; // ['PCH->[SP--]', 'PCL->[SP--],', 'PCH->[SP--](KEEPAC)', 'PCL->[SP--](KEEPAC)']
L[260] = 9'b00_10_00111; // KEEP_AC
L[261] = 9'b10_00_10011; // [PC]:T->PC
L[262] = 9'bxx_xx_xxxxx; // []
L[263] = 9'bxx_xx_xxxxx; // []
L[264] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[265] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[266] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[267] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[268] = 9'b00_01_01011; // [AX]->AH,T->AL
L[269] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[270] = 9'bxx_xx_xxxxx; // []
L[271] = 9'bxx_xx_xxxxx; // []
L[272] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[273] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[274] = 9'bxx_xx_xxxxx; // []
L[275] = 9'bxx_xx_xxxxx; // []
L[276] = 9'bxx_xx_xxxxx; // []
L[277] = 9'bxx_xx_xxxxx; // []
L[278] = 9'bxx_xx_xxxxx; // []
L[279] = 9'bxx_xx_xxxxx; // []
L[280] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[281] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[282] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[283] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[284] = 9'b00_01_01011; // [AX]->AH,T->AL
L[285] = 9'b00_01_00011; // [AX]->T
L[286] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[287] = 9'b10_01_00101; // T->[AX]
L[288] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[289] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[290] = 9'bxx_xx_xxxxx; // []
L[291] = 9'bxx_xx_xxxxx; // []
L[292] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[293] = 9'bxx_xx_xxxxx; // []
L[294] = 9'bxx_xx_xxxxx; // []
L[295] = 9'bxx_xx_xxxxx; // []
L[296] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[297] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[298] = 9'bxx_xx_xxxxx; // []
L[299] = 9'bxx_xx_xxxxx; // []
L[300] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[301] = 9'bxx_xx_xxxxx; // []
L[302] = 9'bxx_xx_xxxxx; // []
L[303] = 9'bxx_xx_xxxxx; // []
L[304] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[305] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[306] = 9'bxx_xx_xxxxx; // []
L[307] = 9'bxx_xx_xxxxx; // []
L[308] = 9'b00_01_00011; // [AX]->T
L[309] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[310] = 9'b10_01_00101; // T->[AX]
L[311] = 9'bxx_xx_xxxxx; // []
L[312] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[313] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[314] = 9'bxx_xx_xxxxx; // []
L[315] = 9'bxx_xx_xxxxx; // []
L[316] = 9'b00_01_00011; // [AX]->T
L[317] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[318] = 9'b10_01_00101; // T->[AX]
L[319] = 9'bxx_xx_xxxxx; // []
L[320] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[321] = 9'b00_00_00011; // [PC]->
L[322] = 9'b00_10_10000; // ['SP++', 'SP+1->SP', '[SP]->T,SP+1->SP']
L[323] = 9'b10_10_00010; // ['ALU([SP])->A', '[SP]->P']
L[324] = 9'bxx_xx_xxxxx; // []
L[325] = 9'bxx_xx_xxxxx; // []
L[326] = 9'bxx_xx_xxxxx; // []
L[327] = 9'bxx_xx_xxxxx; // []
L[328] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[329] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[330] = 9'bxx_xx_xxxxx; // []
L[331] = 9'bxx_xx_xxxxx; // []
L[332] = 9'bxx_xx_xxxxx; // []
L[333] = 9'bxx_xx_xxxxx; // []
L[334] = 9'bxx_xx_xxxxx; // []
L[335] = 9'bxx_xx_xxxxx; // []
L[336] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[337] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[338] = 9'bxx_xx_xxxxx; // []
L[339] = 9'bxx_xx_xxxxx; // []
L[340] = 9'bxx_xx_xxxxx; // []
L[341] = 9'bxx_xx_xxxxx; // []
L[342] = 9'bxx_xx_xxxxx; // []
L[343] = 9'bxx_xx_xxxxx; // []
L[344] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[345] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[346] = 9'bxx_xx_xxxxx; // []
L[347] = 9'bxx_xx_xxxxx; // []
L[348] = 9'bxx_xx_xxxxx; // []
L[349] = 9'bxx_xx_xxxxx; // []
L[350] = 9'bxx_xx_xxxxx; // []
L[351] = 9'bxx_xx_xxxxx; // []
L[352] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[353] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[354] = 9'b11_00_00001; // [PC++]->AH
L[355] = 9'bxx_xx_xxxxx; // []
L[356] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[357] = 9'bxx_xx_xxxxx; // []
L[358] = 9'bxx_xx_xxxxx; // []
L[359] = 9'bxx_xx_xxxxx; // []
L[360] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[361] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[362] = 9'b11_00_00001; // [PC++]->AH
L[363] = 9'bxx_xx_xxxxx; // []
L[364] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[365] = 9'bxx_xx_xxxxx; // []
L[366] = 9'bxx_xx_xxxxx; // []
L[367] = 9'bxx_xx_xxxxx; // []
L[368] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[369] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[370] = 9'b11_00_00001; // [PC++]->AH
L[371] = 9'bxx_xx_xxxxx; // []
L[372] = 9'b00_01_00011; // [AX]->T
L[373] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[374] = 9'b10_01_00101; // T->[AX]
L[375] = 9'bxx_xx_xxxxx; // []
L[376] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[377] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[378] = 9'b11_00_00001; // [PC++]->AH
L[379] = 9'bxx_xx_xxxxx; // []
L[380] = 9'b00_01_00011; // [AX]->T
L[381] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[382] = 9'b10_01_00101; // T->[AX]
L[383] = 9'bxx_xx_xxxxx; // []
L[384] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[385] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[386] = 9'b11_00_10001; // PC+T->PC
L[387] = 9'bxx_xx_xxxxx; // []
L[388] = 9'b10_00_00011; // ['NO-OP', '']
L[389] = 9'bxx_xx_xxxxx; // []
L[390] = 9'bxx_xx_xxxxx; // []
L[391] = 9'bxx_xx_xxxxx; // []
L[392] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[393] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[394] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[395] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[396] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[397] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[398] = 9'bxx_xx_xxxxx; // []
L[399] = 9'bxx_xx_xxxxx; // []
L[400] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[401] = 9'b10_00_00011; // ['NO-OP', '']
L[402] = 9'bxx_xx_xxxxx; // []
L[403] = 9'bxx_xx_xxxxx; // []
L[404] = 9'bxx_xx_xxxxx; // []
L[405] = 9'bxx_xx_xxxxx; // []
L[406] = 9'bxx_xx_xxxxx; // []
L[407] = 9'bxx_xx_xxxxx; // []
L[408] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[409] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[410] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[411] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[412] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[413] = 9'b00_01_00011; // [AX]->T
L[414] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[415] = 9'b10_01_00101; // T->[AX]
L[416] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[417] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[418] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[419] = 9'bxx_xx_xxxxx; // []
L[420] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[421] = 9'bxx_xx_xxxxx; // []
L[422] = 9'bxx_xx_xxxxx; // []
L[423] = 9'bxx_xx_xxxxx; // []
L[424] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[425] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[426] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[427] = 9'bxx_xx_xxxxx; // []
L[428] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[429] = 9'bxx_xx_xxxxx; // []
L[430] = 9'bxx_xx_xxxxx; // []
L[431] = 9'bxx_xx_xxxxx; // []
L[432] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[433] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[434] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[435] = 9'bxx_xx_xxxxx; // []
L[436] = 9'b00_01_00011; // [AX]->T
L[437] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[438] = 9'b10_01_00101; // T->[AX]
L[439] = 9'bxx_xx_xxxxx; // []
L[440] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[441] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[442] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[443] = 9'bxx_xx_xxxxx; // []
L[444] = 9'b00_01_00011; // [AX]->T
L[445] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[446] = 9'b10_01_00101; // T->[AX]
L[447] = 9'bxx_xx_xxxxx; // []
L[448] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[449] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[450] = 9'bxx_xx_xxxxx; // []
L[451] = 9'bxx_xx_xxxxx; // []
L[452] = 9'bxx_xx_xxxxx; // []
L[453] = 9'bxx_xx_xxxxx; // []
L[454] = 9'bxx_xx_xxxxx; // []
L[455] = 9'bxx_xx_xxxxx; // []
L[456] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[457] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[458] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[459] = 9'bxx_xx_xxxxx; // []
L[460] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[461] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[462] = 9'bxx_xx_xxxxx; // []
L[463] = 9'bxx_xx_xxxxx; // []
L[464] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[465] = 9'b10_00_00011; // ['NO-OP', '']
L[466] = 9'bxx_xx_xxxxx; // []
L[467] = 9'bxx_xx_xxxxx; // []
L[468] = 9'bxx_xx_xxxxx; // []
L[469] = 9'bxx_xx_xxxxx; // []
L[470] = 9'bxx_xx_xxxxx; // []
L[471] = 9'bxx_xx_xxxxx; // []
L[472] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[473] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[474] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[475] = 9'bxx_xx_xxxxx; // []
L[476] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[477] = 9'b00_01_00011; // [AX]->T
L[478] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[479] = 9'b10_01_00101; // T->[AX]
L[480] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[481] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[482] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[483] = 9'bxx_xx_xxxxx; // []
L[484] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[485] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[486] = 9'bxx_xx_xxxxx; // []
L[487] = 9'bxx_xx_xxxxx; // []
L[488] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[489] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[490] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[491] = 9'bxx_xx_xxxxx; // []
L[492] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[493] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[494] = 9'bxx_xx_xxxxx; // []
L[495] = 9'bxx_xx_xxxxx; // []
L[496] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[497] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[498] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[499] = 9'bxx_xx_xxxxx; // []
L[500] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[501] = 9'b00_01_00011; // [AX]->T
L[502] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[503] = 9'b10_01_00101; // T->[AX]
L[504] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[505] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[506] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[507] = 9'bxx_xx_xxxxx; // []
L[508] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[509] = 9'b00_01_00011; // [AX]->T
L[510] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[511] = 9'b10_01_00101; // T->[AX]
L[512] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[513] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[514] = 9'b00_10_10000; // ['SP++', 'SP+1->SP', '[SP]->T,SP+1->SP']
L[515] = 9'b00_10_10110; // [SP]->P,SP+1->SP
L[516] = 9'b00_10_10000; // ['SP++', 'SP+1->SP', '[SP]->T,SP+1->SP']
L[517] = 9'b10_10_10011; // [SP]:T->PC
L[518] = 9'bxx_xx_xxxxx; // []
L[519] = 9'bxx_xx_xxxxx; // []
L[520] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[521] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[522] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[523] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[524] = 9'b00_01_01011; // [AX]->AH,T->AL
L[525] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[526] = 9'bxx_xx_xxxxx; // []
L[527] = 9'bxx_xx_xxxxx; // []
L[528] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[529] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[530] = 9'bxx_xx_xxxxx; // []
L[531] = 9'bxx_xx_xxxxx; // []
L[532] = 9'bxx_xx_xxxxx; // []
L[533] = 9'bxx_xx_xxxxx; // []
L[534] = 9'bxx_xx_xxxxx; // []
L[535] = 9'bxx_xx_xxxxx; // []
L[536] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[537] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[538] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[539] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[540] = 9'b00_01_01011; // [AX]->AH,T->AL
L[541] = 9'b00_01_00011; // [AX]->T
L[542] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[543] = 9'b10_01_00101; // T->[AX]
L[544] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[545] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[546] = 9'bxx_xx_xxxxx; // []
L[547] = 9'bxx_xx_xxxxx; // []
L[548] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[549] = 9'bxx_xx_xxxxx; // []
L[550] = 9'bxx_xx_xxxxx; // []
L[551] = 9'bxx_xx_xxxxx; // []
L[552] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[553] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[554] = 9'bxx_xx_xxxxx; // []
L[555] = 9'bxx_xx_xxxxx; // []
L[556] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[557] = 9'bxx_xx_xxxxx; // []
L[558] = 9'bxx_xx_xxxxx; // []
L[559] = 9'bxx_xx_xxxxx; // []
L[560] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[561] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[562] = 9'bxx_xx_xxxxx; // []
L[563] = 9'bxx_xx_xxxxx; // []
L[564] = 9'b00_01_00011; // [AX]->T
L[565] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[566] = 9'b10_01_00101; // T->[AX]
L[567] = 9'bxx_xx_xxxxx; // []
L[568] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[569] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[570] = 9'bxx_xx_xxxxx; // []
L[571] = 9'bxx_xx_xxxxx; // []
L[572] = 9'b00_01_00011; // [AX]->T
L[573] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[574] = 9'b10_01_00101; // T->[AX]
L[575] = 9'bxx_xx_xxxxx; // []
L[576] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[577] = 9'b00_00_00011; // [PC]->
L[578] = 9'b10_10_01110; // ALU(A)->[SP--]
L[579] = 9'bxx_xx_xxxxx; // []
L[580] = 9'bxx_xx_xxxxx; // []
L[581] = 9'bxx_xx_xxxxx; // []
L[582] = 9'bxx_xx_xxxxx; // []
L[583] = 9'bxx_xx_xxxxx; // []
L[584] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[585] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[586] = 9'bxx_xx_xxxxx; // []
L[587] = 9'bxx_xx_xxxxx; // []
L[588] = 9'bxx_xx_xxxxx; // []
L[589] = 9'bxx_xx_xxxxx; // []
L[590] = 9'bxx_xx_xxxxx; // []
L[591] = 9'bxx_xx_xxxxx; // []
L[592] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[593] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[594] = 9'bxx_xx_xxxxx; // []
L[595] = 9'bxx_xx_xxxxx; // []
L[596] = 9'bxx_xx_xxxxx; // []
L[597] = 9'bxx_xx_xxxxx; // []
L[598] = 9'bxx_xx_xxxxx; // []
L[599] = 9'bxx_xx_xxxxx; // []
L[600] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[601] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[602] = 9'bxx_xx_xxxxx; // []
L[603] = 9'bxx_xx_xxxxx; // []
L[604] = 9'bxx_xx_xxxxx; // []
L[605] = 9'bxx_xx_xxxxx; // []
L[606] = 9'bxx_xx_xxxxx; // []
L[607] = 9'bxx_xx_xxxxx; // []
L[608] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[609] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[610] = 9'bxx_xx_xxxxx; // []
L[611] = 9'bxx_xx_xxxxx; // []
L[612] = 9'b10_00_10011; // [PC]:T->PC
L[613] = 9'bxx_xx_xxxxx; // []
L[614] = 9'bxx_xx_xxxxx; // []
L[615] = 9'bxx_xx_xxxxx; // []
L[616] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[617] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[618] = 9'b11_00_00001; // [PC++]->AH
L[619] = 9'bxx_xx_xxxxx; // []
L[620] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[621] = 9'bxx_xx_xxxxx; // []
L[622] = 9'bxx_xx_xxxxx; // []
L[623] = 9'bxx_xx_xxxxx; // []
L[624] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[625] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[626] = 9'b11_00_00001; // [PC++]->AH
L[627] = 9'bxx_xx_xxxxx; // []
L[628] = 9'b00_01_00011; // [AX]->T
L[629] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[630] = 9'b10_01_00101; // T->[AX]
L[631] = 9'bxx_xx_xxxxx; // []
L[632] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[633] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[634] = 9'b11_00_00001; // [PC++]->AH
L[635] = 9'bxx_xx_xxxxx; // []
L[636] = 9'b00_01_00011; // [AX]->T
L[637] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[638] = 9'b10_01_00101; // T->[AX]
L[639] = 9'bxx_xx_xxxxx; // []
L[640] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[641] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[642] = 9'b11_00_10001; // PC+T->PC
L[643] = 9'bxx_xx_xxxxx; // []
L[644] = 9'b10_00_00011; // ['NO-OP', '']
L[645] = 9'bxx_xx_xxxxx; // []
L[646] = 9'bxx_xx_xxxxx; // []
L[647] = 9'bxx_xx_xxxxx; // []
L[648] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[649] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[650] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[651] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[652] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[653] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[654] = 9'bxx_xx_xxxxx; // []
L[655] = 9'bxx_xx_xxxxx; // []
L[656] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[657] = 9'b10_00_00011; // ['NO-OP', '']
L[658] = 9'bxx_xx_xxxxx; // []
L[659] = 9'bxx_xx_xxxxx; // []
L[660] = 9'bxx_xx_xxxxx; // []
L[661] = 9'bxx_xx_xxxxx; // []
L[662] = 9'bxx_xx_xxxxx; // []
L[663] = 9'bxx_xx_xxxxx; // []
L[664] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[665] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[666] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[667] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[668] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[669] = 9'b00_01_00011; // [AX]->T
L[670] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[671] = 9'b10_01_00101; // T->[AX]
L[672] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[673] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[674] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[675] = 9'bxx_xx_xxxxx; // []
L[676] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[677] = 9'bxx_xx_xxxxx; // []
L[678] = 9'bxx_xx_xxxxx; // []
L[679] = 9'bxx_xx_xxxxx; // []
L[680] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[681] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[682] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[683] = 9'bxx_xx_xxxxx; // []
L[684] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[685] = 9'bxx_xx_xxxxx; // []
L[686] = 9'bxx_xx_xxxxx; // []
L[687] = 9'bxx_xx_xxxxx; // []
L[688] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[689] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[690] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[691] = 9'bxx_xx_xxxxx; // []
L[692] = 9'b00_01_00011; // [AX]->T
L[693] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[694] = 9'b10_01_00101; // T->[AX]
L[695] = 9'bxx_xx_xxxxx; // []
L[696] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[697] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[698] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[699] = 9'bxx_xx_xxxxx; // []
L[700] = 9'b00_01_00011; // [AX]->T
L[701] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[702] = 9'b10_01_00101; // T->[AX]
L[703] = 9'bxx_xx_xxxxx; // []
L[704] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[705] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[706] = 9'bxx_xx_xxxxx; // []
L[707] = 9'bxx_xx_xxxxx; // []
L[708] = 9'bxx_xx_xxxxx; // []
L[709] = 9'bxx_xx_xxxxx; // []
L[710] = 9'bxx_xx_xxxxx; // []
L[711] = 9'bxx_xx_xxxxx; // []
L[712] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[713] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[714] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[715] = 9'bxx_xx_xxxxx; // []
L[716] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[717] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[718] = 9'bxx_xx_xxxxx; // []
L[719] = 9'bxx_xx_xxxxx; // []
L[720] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[721] = 9'b10_00_00011; // ['NO-OP', '']
L[722] = 9'bxx_xx_xxxxx; // []
L[723] = 9'bxx_xx_xxxxx; // []
L[724] = 9'bxx_xx_xxxxx; // []
L[725] = 9'bxx_xx_xxxxx; // []
L[726] = 9'bxx_xx_xxxxx; // []
L[727] = 9'bxx_xx_xxxxx; // []
L[728] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[729] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[730] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[731] = 9'bxx_xx_xxxxx; // []
L[732] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[733] = 9'b00_01_00011; // [AX]->T
L[734] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[735] = 9'b10_01_00101; // T->[AX]
L[736] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[737] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[738] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[739] = 9'bxx_xx_xxxxx; // []
L[740] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[741] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[742] = 9'bxx_xx_xxxxx; // []
L[743] = 9'bxx_xx_xxxxx; // []
L[744] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[745] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[746] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[747] = 9'bxx_xx_xxxxx; // []
L[748] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[749] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[750] = 9'bxx_xx_xxxxx; // []
L[751] = 9'bxx_xx_xxxxx; // []
L[752] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[753] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[754] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[755] = 9'bxx_xx_xxxxx; // []
L[756] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[757] = 9'b00_01_00011; // [AX]->T
L[758] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[759] = 9'b10_01_00101; // T->[AX]
L[760] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[761] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[762] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[763] = 9'bxx_xx_xxxxx; // []
L[764] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[765] = 9'b00_01_00011; // [AX]->T
L[766] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[767] = 9'b10_01_00101; // T->[AX]
L[768] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[769] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[770] = 9'b11_10_10000; // SP+1->SP
L[771] = 9'bxx_xx_xxxxx; // []
L[772] = 9'b00_10_10000; // ['SP++', 'SP+1->SP', '[SP]->T,SP+1->SP']
L[773] = 9'b00_10_10011; // [SP]:T->PC
L[774] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[775] = 9'bxx_xx_xxxxx; // []
L[776] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[777] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[778] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[779] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[780] = 9'b00_01_01011; // [AX]->AH,T->AL
L[781] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[782] = 9'bxx_xx_xxxxx; // []
L[783] = 9'bxx_xx_xxxxx; // []
L[784] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[785] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[786] = 9'bxx_xx_xxxxx; // []
L[787] = 9'bxx_xx_xxxxx; // []
L[788] = 9'bxx_xx_xxxxx; // []
L[789] = 9'bxx_xx_xxxxx; // []
L[790] = 9'bxx_xx_xxxxx; // []
L[791] = 9'bxx_xx_xxxxx; // []
L[792] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[793] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[794] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[795] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[796] = 9'b00_01_01011; // [AX]->AH,T->AL
L[797] = 9'b00_01_00011; // [AX]->T
L[798] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[799] = 9'b10_01_00101; // T->[AX]
L[800] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[801] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[802] = 9'bxx_xx_xxxxx; // []
L[803] = 9'bxx_xx_xxxxx; // []
L[804] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[805] = 9'bxx_xx_xxxxx; // []
L[806] = 9'bxx_xx_xxxxx; // []
L[807] = 9'bxx_xx_xxxxx; // []
L[808] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[809] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[810] = 9'bxx_xx_xxxxx; // []
L[811] = 9'bxx_xx_xxxxx; // []
L[812] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[813] = 9'bxx_xx_xxxxx; // []
L[814] = 9'bxx_xx_xxxxx; // []
L[815] = 9'bxx_xx_xxxxx; // []
L[816] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[817] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[818] = 9'bxx_xx_xxxxx; // []
L[819] = 9'bxx_xx_xxxxx; // []
L[820] = 9'b00_01_00011; // [AX]->T
L[821] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[822] = 9'b10_01_00101; // T->[AX]
L[823] = 9'bxx_xx_xxxxx; // []
L[824] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[825] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[826] = 9'bxx_xx_xxxxx; // []
L[827] = 9'bxx_xx_xxxxx; // []
L[828] = 9'b00_01_00011; // [AX]->T
L[829] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[830] = 9'b10_01_00101; // T->[AX]
L[831] = 9'bxx_xx_xxxxx; // []
L[832] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[833] = 9'b00_00_00011; // [PC]->
L[834] = 9'b00_10_10000; // ['SP++', 'SP+1->SP', '[SP]->T,SP+1->SP']
L[835] = 9'b10_10_00010; // ['ALU([SP])->A', '[SP]->P']
L[836] = 9'bxx_xx_xxxxx; // []
L[837] = 9'bxx_xx_xxxxx; // []
L[838] = 9'bxx_xx_xxxxx; // []
L[839] = 9'bxx_xx_xxxxx; // []
L[840] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[841] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[842] = 9'bxx_xx_xxxxx; // []
L[843] = 9'bxx_xx_xxxxx; // []
L[844] = 9'bxx_xx_xxxxx; // []
L[845] = 9'bxx_xx_xxxxx; // []
L[846] = 9'bxx_xx_xxxxx; // []
L[847] = 9'bxx_xx_xxxxx; // []
L[848] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[849] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[850] = 9'bxx_xx_xxxxx; // []
L[851] = 9'bxx_xx_xxxxx; // []
L[852] = 9'bxx_xx_xxxxx; // []
L[853] = 9'bxx_xx_xxxxx; // []
L[854] = 9'bxx_xx_xxxxx; // []
L[855] = 9'bxx_xx_xxxxx; // []
L[856] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[857] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[858] = 9'bxx_xx_xxxxx; // []
L[859] = 9'bxx_xx_xxxxx; // []
L[860] = 9'bxx_xx_xxxxx; // []
L[861] = 9'bxx_xx_xxxxx; // []
L[862] = 9'bxx_xx_xxxxx; // []
L[863] = 9'bxx_xx_xxxxx; // []
L[864] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[865] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[866] = 9'b00_00_00001; // [PC++]->AH
L[867] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[868] = 9'b10_01_10011; // [AX]:T->PC
L[869] = 9'bxx_xx_xxxxx; // []
L[870] = 9'bxx_xx_xxxxx; // []
L[871] = 9'bxx_xx_xxxxx; // []
L[872] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[873] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[874] = 9'b11_00_00001; // [PC++]->AH
L[875] = 9'bxx_xx_xxxxx; // []
L[876] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[877] = 9'bxx_xx_xxxxx; // []
L[878] = 9'bxx_xx_xxxxx; // []
L[879] = 9'bxx_xx_xxxxx; // []
L[880] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[881] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[882] = 9'b11_00_00001; // [PC++]->AH
L[883] = 9'bxx_xx_xxxxx; // []
L[884] = 9'b00_01_00011; // [AX]->T
L[885] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[886] = 9'b10_01_00101; // T->[AX]
L[887] = 9'bxx_xx_xxxxx; // []
L[888] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[889] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[890] = 9'b11_00_00001; // [PC++]->AH
L[891] = 9'bxx_xx_xxxxx; // []
L[892] = 9'b00_01_00011; // [AX]->T
L[893] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[894] = 9'b10_01_00101; // T->[AX]
L[895] = 9'bxx_xx_xxxxx; // []
L[896] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[897] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[898] = 9'b11_00_10001; // PC+T->PC
L[899] = 9'bxx_xx_xxxxx; // []
L[900] = 9'b10_00_00011; // ['NO-OP', '']
L[901] = 9'bxx_xx_xxxxx; // []
L[902] = 9'bxx_xx_xxxxx; // []
L[903] = 9'bxx_xx_xxxxx; // []
L[904] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[905] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[906] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[907] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[908] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[909] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[910] = 9'bxx_xx_xxxxx; // []
L[911] = 9'bxx_xx_xxxxx; // []
L[912] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[913] = 9'b10_00_00011; // ['NO-OP', '']
L[914] = 9'bxx_xx_xxxxx; // []
L[915] = 9'bxx_xx_xxxxx; // []
L[916] = 9'bxx_xx_xxxxx; // []
L[917] = 9'bxx_xx_xxxxx; // []
L[918] = 9'bxx_xx_xxxxx; // []
L[919] = 9'bxx_xx_xxxxx; // []
L[920] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[921] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[922] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[923] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[924] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[925] = 9'b00_01_00011; // [AX]->T
L[926] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[927] = 9'b10_01_00101; // T->[AX]
L[928] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[929] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[930] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[931] = 9'bxx_xx_xxxxx; // []
L[932] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[933] = 9'bxx_xx_xxxxx; // []
L[934] = 9'bxx_xx_xxxxx; // []
L[935] = 9'bxx_xx_xxxxx; // []
L[936] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[937] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[938] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[939] = 9'bxx_xx_xxxxx; // []
L[940] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[941] = 9'bxx_xx_xxxxx; // []
L[942] = 9'bxx_xx_xxxxx; // []
L[943] = 9'bxx_xx_xxxxx; // []
L[944] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[945] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[946] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[947] = 9'bxx_xx_xxxxx; // []
L[948] = 9'b00_01_00011; // [AX]->T
L[949] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[950] = 9'b10_01_00101; // T->[AX]
L[951] = 9'bxx_xx_xxxxx; // []
L[952] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[953] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[954] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[955] = 9'bxx_xx_xxxxx; // []
L[956] = 9'b00_01_00011; // [AX]->T
L[957] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[958] = 9'b10_01_00101; // T->[AX]
L[959] = 9'bxx_xx_xxxxx; // []
L[960] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[961] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[962] = 9'bxx_xx_xxxxx; // []
L[963] = 9'bxx_xx_xxxxx; // []
L[964] = 9'bxx_xx_xxxxx; // []
L[965] = 9'bxx_xx_xxxxx; // []
L[966] = 9'bxx_xx_xxxxx; // []
L[967] = 9'bxx_xx_xxxxx; // []
L[968] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[969] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[970] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[971] = 9'bxx_xx_xxxxx; // []
L[972] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[973] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[974] = 9'bxx_xx_xxxxx; // []
L[975] = 9'bxx_xx_xxxxx; // []
L[976] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[977] = 9'b10_00_00011; // ['NO-OP', '']
L[978] = 9'bxx_xx_xxxxx; // []
L[979] = 9'bxx_xx_xxxxx; // []
L[980] = 9'bxx_xx_xxxxx; // []
L[981] = 9'bxx_xx_xxxxx; // []
L[982] = 9'bxx_xx_xxxxx; // []
L[983] = 9'bxx_xx_xxxxx; // []
L[984] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[985] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[986] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[987] = 9'bxx_xx_xxxxx; // []
L[988] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[989] = 9'b00_01_00011; // [AX]->T
L[990] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[991] = 9'b10_01_00101; // T->[AX]
L[992] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[993] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[994] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[995] = 9'bxx_xx_xxxxx; // []
L[996] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[997] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[998] = 9'bxx_xx_xxxxx; // []
L[999] = 9'bxx_xx_xxxxx; // []
L[1000] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1001] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1002] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1003] = 9'bxx_xx_xxxxx; // []
L[1004] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1005] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1006] = 9'bxx_xx_xxxxx; // []
L[1007] = 9'bxx_xx_xxxxx; // []
L[1008] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1009] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1010] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1011] = 9'bxx_xx_xxxxx; // []
L[1012] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1013] = 9'b00_01_00011; // [AX]->T
L[1014] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1015] = 9'b10_01_00101; // T->[AX]
L[1016] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1017] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1018] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1019] = 9'bxx_xx_xxxxx; // []
L[1020] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1021] = 9'b00_01_00011; // [AX]->T
L[1022] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1023] = 9'b10_01_00101; // T->[AX]
L[1024] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1025] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1026] = 9'bxx_xx_xxxxx; // []
L[1027] = 9'bxx_xx_xxxxx; // []
L[1028] = 9'bxx_xx_xxxxx; // []
L[1029] = 9'bxx_xx_xxxxx; // []
L[1030] = 9'bxx_xx_xxxxx; // []
L[1031] = 9'bxx_xx_xxxxx; // []
L[1032] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1033] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1034] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1035] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1036] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1037] = 9'b10_01_00110; // ALU()->[AX]
L[1038] = 9'bxx_xx_xxxxx; // []
L[1039] = 9'bxx_xx_xxxxx; // []
L[1040] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1041] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1042] = 9'bxx_xx_xxxxx; // []
L[1043] = 9'bxx_xx_xxxxx; // []
L[1044] = 9'bxx_xx_xxxxx; // []
L[1045] = 9'bxx_xx_xxxxx; // []
L[1046] = 9'bxx_xx_xxxxx; // []
L[1047] = 9'bxx_xx_xxxxx; // []
L[1048] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1049] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1050] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1051] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1052] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1053] = 9'b10_01_00110; // ALU()->[AX]
L[1054] = 9'bxx_xx_xxxxx; // []
L[1055] = 9'bxx_xx_xxxxx; // []
L[1056] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1057] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1058] = 9'bxx_xx_xxxxx; // []
L[1059] = 9'bxx_xx_xxxxx; // []
L[1060] = 9'b10_01_00110; // ALU()->[AX]
L[1061] = 9'bxx_xx_xxxxx; // []
L[1062] = 9'bxx_xx_xxxxx; // []
L[1063] = 9'bxx_xx_xxxxx; // []
L[1064] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1065] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1066] = 9'bxx_xx_xxxxx; // []
L[1067] = 9'bxx_xx_xxxxx; // []
L[1068] = 9'b10_01_00110; // ALU()->[AX]
L[1069] = 9'bxx_xx_xxxxx; // []
L[1070] = 9'bxx_xx_xxxxx; // []
L[1071] = 9'bxx_xx_xxxxx; // []
L[1072] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1073] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1074] = 9'bxx_xx_xxxxx; // []
L[1075] = 9'bxx_xx_xxxxx; // []
L[1076] = 9'b10_01_00110; // ALU()->[AX]
L[1077] = 9'bxx_xx_xxxxx; // []
L[1078] = 9'bxx_xx_xxxxx; // []
L[1079] = 9'bxx_xx_xxxxx; // []
L[1080] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1081] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1082] = 9'bxx_xx_xxxxx; // []
L[1083] = 9'bxx_xx_xxxxx; // []
L[1084] = 9'b10_01_00110; // ALU()->[AX]
L[1085] = 9'bxx_xx_xxxxx; // []
L[1086] = 9'bxx_xx_xxxxx; // []
L[1087] = 9'bxx_xx_xxxxx; // []
L[1088] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1089] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1090] = 9'bxx_xx_xxxxx; // []
L[1091] = 9'bxx_xx_xxxxx; // []
L[1092] = 9'bxx_xx_xxxxx; // []
L[1093] = 9'bxx_xx_xxxxx; // []
L[1094] = 9'bxx_xx_xxxxx; // []
L[1095] = 9'bxx_xx_xxxxx; // []
L[1096] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1097] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1098] = 9'bxx_xx_xxxxx; // []
L[1099] = 9'bxx_xx_xxxxx; // []
L[1100] = 9'bxx_xx_xxxxx; // []
L[1101] = 9'bxx_xx_xxxxx; // []
L[1102] = 9'bxx_xx_xxxxx; // []
L[1103] = 9'bxx_xx_xxxxx; // []
L[1104] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1105] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1106] = 9'bxx_xx_xxxxx; // []
L[1107] = 9'bxx_xx_xxxxx; // []
L[1108] = 9'bxx_xx_xxxxx; // []
L[1109] = 9'bxx_xx_xxxxx; // []
L[1110] = 9'bxx_xx_xxxxx; // []
L[1111] = 9'bxx_xx_xxxxx; // []
L[1112] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1113] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1114] = 9'bxx_xx_xxxxx; // []
L[1115] = 9'bxx_xx_xxxxx; // []
L[1116] = 9'bxx_xx_xxxxx; // []
L[1117] = 9'bxx_xx_xxxxx; // []
L[1118] = 9'bxx_xx_xxxxx; // []
L[1119] = 9'bxx_xx_xxxxx; // []
L[1120] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1121] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1122] = 9'b11_00_00001; // [PC++]->AH
L[1123] = 9'bxx_xx_xxxxx; // []
L[1124] = 9'b10_01_00110; // ALU()->[AX]
L[1125] = 9'bxx_xx_xxxxx; // []
L[1126] = 9'bxx_xx_xxxxx; // []
L[1127] = 9'bxx_xx_xxxxx; // []
L[1128] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1129] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1130] = 9'b11_00_00001; // [PC++]->AH
L[1131] = 9'bxx_xx_xxxxx; // []
L[1132] = 9'b10_01_00110; // ALU()->[AX]
L[1133] = 9'bxx_xx_xxxxx; // []
L[1134] = 9'bxx_xx_xxxxx; // []
L[1135] = 9'bxx_xx_xxxxx; // []
L[1136] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1137] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1138] = 9'b11_00_00001; // [PC++]->AH
L[1139] = 9'bxx_xx_xxxxx; // []
L[1140] = 9'b10_01_00110; // ALU()->[AX]
L[1141] = 9'bxx_xx_xxxxx; // []
L[1142] = 9'bxx_xx_xxxxx; // []
L[1143] = 9'bxx_xx_xxxxx; // []
L[1144] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1145] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1146] = 9'b11_00_00001; // [PC++]->AH
L[1147] = 9'bxx_xx_xxxxx; // []
L[1148] = 9'b10_01_00110; // ALU()->[AX]
L[1149] = 9'bxx_xx_xxxxx; // []
L[1150] = 9'bxx_xx_xxxxx; // []
L[1151] = 9'bxx_xx_xxxxx; // []
L[1152] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1153] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1154] = 9'b11_00_10001; // PC+T->PC
L[1155] = 9'bxx_xx_xxxxx; // []
L[1156] = 9'b10_00_00011; // ['NO-OP', '']
L[1157] = 9'bxx_xx_xxxxx; // []
L[1158] = 9'bxx_xx_xxxxx; // []
L[1159] = 9'bxx_xx_xxxxx; // []
L[1160] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1161] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1162] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1163] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[1164] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1165] = 9'b10_01_00110; // ALU()->[AX]
L[1166] = 9'bxx_xx_xxxxx; // []
L[1167] = 9'bxx_xx_xxxxx; // []
L[1168] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1169] = 9'b10_00_00011; // ['NO-OP', '']
L[1170] = 9'bxx_xx_xxxxx; // []
L[1171] = 9'bxx_xx_xxxxx; // []
L[1172] = 9'bxx_xx_xxxxx; // []
L[1173] = 9'bxx_xx_xxxxx; // []
L[1174] = 9'bxx_xx_xxxxx; // []
L[1175] = 9'bxx_xx_xxxxx; // []
L[1176] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1177] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1178] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1179] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[1180] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1181] = 9'b10_01_00110; // ALU()->[AX]
L[1182] = 9'bxx_xx_xxxxx; // []
L[1183] = 9'bxx_xx_xxxxx; // []
L[1184] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1185] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1186] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1187] = 9'bxx_xx_xxxxx; // []
L[1188] = 9'b10_01_00110; // ALU()->[AX]
L[1189] = 9'bxx_xx_xxxxx; // []
L[1190] = 9'bxx_xx_xxxxx; // []
L[1191] = 9'bxx_xx_xxxxx; // []
L[1192] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1193] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1194] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1195] = 9'bxx_xx_xxxxx; // []
L[1196] = 9'b10_01_00110; // ALU()->[AX]
L[1197] = 9'bxx_xx_xxxxx; // []
L[1198] = 9'bxx_xx_xxxxx; // []
L[1199] = 9'bxx_xx_xxxxx; // []
L[1200] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1201] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1202] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1203] = 9'bxx_xx_xxxxx; // []
L[1204] = 9'b10_01_00110; // ALU()->[AX]
L[1205] = 9'bxx_xx_xxxxx; // []
L[1206] = 9'bxx_xx_xxxxx; // []
L[1207] = 9'bxx_xx_xxxxx; // []
L[1208] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1209] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1210] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1211] = 9'bxx_xx_xxxxx; // []
L[1212] = 9'b10_01_00110; // ALU()->[AX]
L[1213] = 9'bxx_xx_xxxxx; // []
L[1214] = 9'bxx_xx_xxxxx; // []
L[1215] = 9'bxx_xx_xxxxx; // []
L[1216] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1217] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1218] = 9'bxx_xx_xxxxx; // []
L[1219] = 9'bxx_xx_xxxxx; // []
L[1220] = 9'bxx_xx_xxxxx; // []
L[1221] = 9'bxx_xx_xxxxx; // []
L[1222] = 9'bxx_xx_xxxxx; // []
L[1223] = 9'bxx_xx_xxxxx; // []
L[1224] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1225] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1226] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1227] = 9'bxx_xx_xxxxx; // []
L[1228] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1229] = 9'b10_01_00110; // ALU()->[AX]
L[1230] = 9'bxx_xx_xxxxx; // []
L[1231] = 9'bxx_xx_xxxxx; // []
L[1232] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1233] = 9'b10_00_10010; // X->S
L[1234] = 9'bxx_xx_xxxxx; // []
L[1235] = 9'bxx_xx_xxxxx; // []
L[1236] = 9'bxx_xx_xxxxx; // []
L[1237] = 9'bxx_xx_xxxxx; // []
L[1238] = 9'bxx_xx_xxxxx; // []
L[1239] = 9'bxx_xx_xxxxx; // []
L[1240] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1241] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1242] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1243] = 9'bxx_xx_xxxxx; // []
L[1244] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1245] = 9'b10_01_00110; // ALU()->[AX]
L[1246] = 9'bxx_xx_xxxxx; // []
L[1247] = 9'bxx_xx_xxxxx; // []
L[1248] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1249] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1250] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1251] = 9'bxx_xx_xxxxx; // []
L[1252] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1253] = 9'b10_01_00110; // ALU()->[AX]
L[1254] = 9'bxx_xx_xxxxx; // []
L[1255] = 9'bxx_xx_xxxxx; // []
L[1256] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1257] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1258] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1259] = 9'bxx_xx_xxxxx; // []
L[1260] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1261] = 9'b10_01_00110; // ALU()->[AX]
L[1262] = 9'bxx_xx_xxxxx; // []
L[1263] = 9'bxx_xx_xxxxx; // []
L[1264] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1265] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1266] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1267] = 9'bxx_xx_xxxxx; // []
L[1268] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1269] = 9'b10_01_00110; // ALU()->[AX]
L[1270] = 9'bxx_xx_xxxxx; // []
L[1271] = 9'bxx_xx_xxxxx; // []
L[1272] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1273] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1274] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1275] = 9'bxx_xx_xxxxx; // []
L[1276] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1277] = 9'b10_01_00110; // ALU()->[AX]
L[1278] = 9'bxx_xx_xxxxx; // []
L[1279] = 9'bxx_xx_xxxxx; // []
L[1280] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1281] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1282] = 9'bxx_xx_xxxxx; // []
L[1283] = 9'bxx_xx_xxxxx; // []
L[1284] = 9'bxx_xx_xxxxx; // []
L[1285] = 9'bxx_xx_xxxxx; // []
L[1286] = 9'bxx_xx_xxxxx; // []
L[1287] = 9'bxx_xx_xxxxx; // []
L[1288] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1289] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1290] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1291] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1292] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1293] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1294] = 9'bxx_xx_xxxxx; // []
L[1295] = 9'bxx_xx_xxxxx; // []
L[1296] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1297] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1298] = 9'bxx_xx_xxxxx; // []
L[1299] = 9'bxx_xx_xxxxx; // []
L[1300] = 9'bxx_xx_xxxxx; // []
L[1301] = 9'bxx_xx_xxxxx; // []
L[1302] = 9'bxx_xx_xxxxx; // []
L[1303] = 9'bxx_xx_xxxxx; // []
L[1304] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1305] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1306] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1307] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1308] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1309] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1310] = 9'bxx_xx_xxxxx; // []
L[1311] = 9'bxx_xx_xxxxx; // []
L[1312] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1313] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1314] = 9'bxx_xx_xxxxx; // []
L[1315] = 9'bxx_xx_xxxxx; // []
L[1316] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1317] = 9'bxx_xx_xxxxx; // []
L[1318] = 9'bxx_xx_xxxxx; // []
L[1319] = 9'bxx_xx_xxxxx; // []
L[1320] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1321] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1322] = 9'bxx_xx_xxxxx; // []
L[1323] = 9'bxx_xx_xxxxx; // []
L[1324] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1325] = 9'bxx_xx_xxxxx; // []
L[1326] = 9'bxx_xx_xxxxx; // []
L[1327] = 9'bxx_xx_xxxxx; // []
L[1328] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1329] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1330] = 9'bxx_xx_xxxxx; // []
L[1331] = 9'bxx_xx_xxxxx; // []
L[1332] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1333] = 9'bxx_xx_xxxxx; // []
L[1334] = 9'bxx_xx_xxxxx; // []
L[1335] = 9'bxx_xx_xxxxx; // []
L[1336] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1337] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1338] = 9'bxx_xx_xxxxx; // []
L[1339] = 9'bxx_xx_xxxxx; // []
L[1340] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1341] = 9'bxx_xx_xxxxx; // []
L[1342] = 9'bxx_xx_xxxxx; // []
L[1343] = 9'bxx_xx_xxxxx; // []
L[1344] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1345] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1346] = 9'bxx_xx_xxxxx; // []
L[1347] = 9'bxx_xx_xxxxx; // []
L[1348] = 9'bxx_xx_xxxxx; // []
L[1349] = 9'bxx_xx_xxxxx; // []
L[1350] = 9'bxx_xx_xxxxx; // []
L[1351] = 9'bxx_xx_xxxxx; // []
L[1352] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1353] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1354] = 9'bxx_xx_xxxxx; // []
L[1355] = 9'bxx_xx_xxxxx; // []
L[1356] = 9'bxx_xx_xxxxx; // []
L[1357] = 9'bxx_xx_xxxxx; // []
L[1358] = 9'bxx_xx_xxxxx; // []
L[1359] = 9'bxx_xx_xxxxx; // []
L[1360] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1361] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1362] = 9'bxx_xx_xxxxx; // []
L[1363] = 9'bxx_xx_xxxxx; // []
L[1364] = 9'bxx_xx_xxxxx; // []
L[1365] = 9'bxx_xx_xxxxx; // []
L[1366] = 9'bxx_xx_xxxxx; // []
L[1367] = 9'bxx_xx_xxxxx; // []
L[1368] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1369] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1370] = 9'bxx_xx_xxxxx; // []
L[1371] = 9'bxx_xx_xxxxx; // []
L[1372] = 9'bxx_xx_xxxxx; // []
L[1373] = 9'bxx_xx_xxxxx; // []
L[1374] = 9'bxx_xx_xxxxx; // []
L[1375] = 9'bxx_xx_xxxxx; // []
L[1376] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1377] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1378] = 9'b11_00_00001; // [PC++]->AH
L[1379] = 9'bxx_xx_xxxxx; // []
L[1380] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1381] = 9'bxx_xx_xxxxx; // []
L[1382] = 9'bxx_xx_xxxxx; // []
L[1383] = 9'bxx_xx_xxxxx; // []
L[1384] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1385] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1386] = 9'b11_00_00001; // [PC++]->AH
L[1387] = 9'bxx_xx_xxxxx; // []
L[1388] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1389] = 9'bxx_xx_xxxxx; // []
L[1390] = 9'bxx_xx_xxxxx; // []
L[1391] = 9'bxx_xx_xxxxx; // []
L[1392] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1393] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1394] = 9'b11_00_00001; // [PC++]->AH
L[1395] = 9'bxx_xx_xxxxx; // []
L[1396] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1397] = 9'bxx_xx_xxxxx; // []
L[1398] = 9'bxx_xx_xxxxx; // []
L[1399] = 9'bxx_xx_xxxxx; // []
L[1400] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1401] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1402] = 9'b11_00_00001; // [PC++]->AH
L[1403] = 9'bxx_xx_xxxxx; // []
L[1404] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1405] = 9'bxx_xx_xxxxx; // []
L[1406] = 9'bxx_xx_xxxxx; // []
L[1407] = 9'bxx_xx_xxxxx; // []
L[1408] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1409] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1410] = 9'b11_00_10001; // PC+T->PC
L[1411] = 9'bxx_xx_xxxxx; // []
L[1412] = 9'b10_00_00011; // ['NO-OP', '']
L[1413] = 9'bxx_xx_xxxxx; // []
L[1414] = 9'bxx_xx_xxxxx; // []
L[1415] = 9'bxx_xx_xxxxx; // []
L[1416] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1417] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1418] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1419] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[1420] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1421] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1422] = 9'bxx_xx_xxxxx; // []
L[1423] = 9'bxx_xx_xxxxx; // []
L[1424] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1425] = 9'b10_00_00011; // ['NO-OP', '']
L[1426] = 9'bxx_xx_xxxxx; // []
L[1427] = 9'bxx_xx_xxxxx; // []
L[1428] = 9'bxx_xx_xxxxx; // []
L[1429] = 9'bxx_xx_xxxxx; // []
L[1430] = 9'bxx_xx_xxxxx; // []
L[1431] = 9'bxx_xx_xxxxx; // []
L[1432] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1433] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1434] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1435] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[1436] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1437] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1438] = 9'bxx_xx_xxxxx; // []
L[1439] = 9'bxx_xx_xxxxx; // []
L[1440] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1441] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1442] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1443] = 9'bxx_xx_xxxxx; // []
L[1444] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1445] = 9'bxx_xx_xxxxx; // []
L[1446] = 9'bxx_xx_xxxxx; // []
L[1447] = 9'bxx_xx_xxxxx; // []
L[1448] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1449] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1450] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1451] = 9'bxx_xx_xxxxx; // []
L[1452] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1453] = 9'bxx_xx_xxxxx; // []
L[1454] = 9'bxx_xx_xxxxx; // []
L[1455] = 9'bxx_xx_xxxxx; // []
L[1456] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1457] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1458] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1459] = 9'bxx_xx_xxxxx; // []
L[1460] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1461] = 9'bxx_xx_xxxxx; // []
L[1462] = 9'bxx_xx_xxxxx; // []
L[1463] = 9'bxx_xx_xxxxx; // []
L[1464] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1465] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1466] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1467] = 9'bxx_xx_xxxxx; // []
L[1468] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1469] = 9'bxx_xx_xxxxx; // []
L[1470] = 9'bxx_xx_xxxxx; // []
L[1471] = 9'bxx_xx_xxxxx; // []
L[1472] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1473] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1474] = 9'bxx_xx_xxxxx; // []
L[1475] = 9'bxx_xx_xxxxx; // []
L[1476] = 9'bxx_xx_xxxxx; // []
L[1477] = 9'bxx_xx_xxxxx; // []
L[1478] = 9'bxx_xx_xxxxx; // []
L[1479] = 9'bxx_xx_xxxxx; // []
L[1480] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1481] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1482] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1483] = 9'bxx_xx_xxxxx; // []
L[1484] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1485] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1486] = 9'bxx_xx_xxxxx; // []
L[1487] = 9'bxx_xx_xxxxx; // []
L[1488] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1489] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1490] = 9'bxx_xx_xxxxx; // []
L[1491] = 9'bxx_xx_xxxxx; // []
L[1492] = 9'bxx_xx_xxxxx; // []
L[1493] = 9'bxx_xx_xxxxx; // []
L[1494] = 9'bxx_xx_xxxxx; // []
L[1495] = 9'bxx_xx_xxxxx; // []
L[1496] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1497] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1498] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1499] = 9'bxx_xx_xxxxx; // []
L[1500] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1501] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1502] = 9'bxx_xx_xxxxx; // []
L[1503] = 9'bxx_xx_xxxxx; // []
L[1504] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1505] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1506] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1507] = 9'bxx_xx_xxxxx; // []
L[1508] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1509] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1510] = 9'bxx_xx_xxxxx; // []
L[1511] = 9'bxx_xx_xxxxx; // []
L[1512] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1513] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1514] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1515] = 9'bxx_xx_xxxxx; // []
L[1516] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1517] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1518] = 9'bxx_xx_xxxxx; // []
L[1519] = 9'bxx_xx_xxxxx; // []
L[1520] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1521] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1522] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1523] = 9'bxx_xx_xxxxx; // []
L[1524] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1525] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1526] = 9'bxx_xx_xxxxx; // []
L[1527] = 9'bxx_xx_xxxxx; // []
L[1528] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1529] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1530] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1531] = 9'bxx_xx_xxxxx; // []
L[1532] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1533] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1534] = 9'bxx_xx_xxxxx; // []
L[1535] = 9'bxx_xx_xxxxx; // []
L[1536] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1537] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1538] = 9'bxx_xx_xxxxx; // []
L[1539] = 9'bxx_xx_xxxxx; // []
L[1540] = 9'bxx_xx_xxxxx; // []
L[1541] = 9'bxx_xx_xxxxx; // []
L[1542] = 9'bxx_xx_xxxxx; // []
L[1543] = 9'bxx_xx_xxxxx; // []
L[1544] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1545] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1546] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1547] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1548] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1549] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1550] = 9'bxx_xx_xxxxx; // []
L[1551] = 9'bxx_xx_xxxxx; // []
L[1552] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1553] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1554] = 9'bxx_xx_xxxxx; // []
L[1555] = 9'bxx_xx_xxxxx; // []
L[1556] = 9'bxx_xx_xxxxx; // []
L[1557] = 9'bxx_xx_xxxxx; // []
L[1558] = 9'bxx_xx_xxxxx; // []
L[1559] = 9'bxx_xx_xxxxx; // []
L[1560] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1561] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1562] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1563] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1564] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1565] = 9'b00_01_00011; // [AX]->T
L[1566] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1567] = 9'b10_01_00101; // T->[AX]
L[1568] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1569] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1570] = 9'bxx_xx_xxxxx; // []
L[1571] = 9'bxx_xx_xxxxx; // []
L[1572] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1573] = 9'bxx_xx_xxxxx; // []
L[1574] = 9'bxx_xx_xxxxx; // []
L[1575] = 9'bxx_xx_xxxxx; // []
L[1576] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1577] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1578] = 9'bxx_xx_xxxxx; // []
L[1579] = 9'bxx_xx_xxxxx; // []
L[1580] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1581] = 9'bxx_xx_xxxxx; // []
L[1582] = 9'bxx_xx_xxxxx; // []
L[1583] = 9'bxx_xx_xxxxx; // []
L[1584] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1585] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1586] = 9'bxx_xx_xxxxx; // []
L[1587] = 9'bxx_xx_xxxxx; // []
L[1588] = 9'b00_01_00011; // [AX]->T
L[1589] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1590] = 9'b10_01_00101; // T->[AX]
L[1591] = 9'bxx_xx_xxxxx; // []
L[1592] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1593] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1594] = 9'bxx_xx_xxxxx; // []
L[1595] = 9'bxx_xx_xxxxx; // []
L[1596] = 9'b00_01_00011; // [AX]->T
L[1597] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1598] = 9'b10_01_00101; // T->[AX]
L[1599] = 9'bxx_xx_xxxxx; // []
L[1600] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1601] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1602] = 9'bxx_xx_xxxxx; // []
L[1603] = 9'bxx_xx_xxxxx; // []
L[1604] = 9'bxx_xx_xxxxx; // []
L[1605] = 9'bxx_xx_xxxxx; // []
L[1606] = 9'bxx_xx_xxxxx; // []
L[1607] = 9'bxx_xx_xxxxx; // []
L[1608] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1609] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1610] = 9'bxx_xx_xxxxx; // []
L[1611] = 9'bxx_xx_xxxxx; // []
L[1612] = 9'bxx_xx_xxxxx; // []
L[1613] = 9'bxx_xx_xxxxx; // []
L[1614] = 9'bxx_xx_xxxxx; // []
L[1615] = 9'bxx_xx_xxxxx; // []
L[1616] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1617] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1618] = 9'bxx_xx_xxxxx; // []
L[1619] = 9'bxx_xx_xxxxx; // []
L[1620] = 9'bxx_xx_xxxxx; // []
L[1621] = 9'bxx_xx_xxxxx; // []
L[1622] = 9'bxx_xx_xxxxx; // []
L[1623] = 9'bxx_xx_xxxxx; // []
L[1624] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1625] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1626] = 9'bxx_xx_xxxxx; // []
L[1627] = 9'bxx_xx_xxxxx; // []
L[1628] = 9'bxx_xx_xxxxx; // []
L[1629] = 9'bxx_xx_xxxxx; // []
L[1630] = 9'bxx_xx_xxxxx; // []
L[1631] = 9'bxx_xx_xxxxx; // []
L[1632] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1633] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1634] = 9'b11_00_00001; // [PC++]->AH
L[1635] = 9'bxx_xx_xxxxx; // []
L[1636] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1637] = 9'bxx_xx_xxxxx; // []
L[1638] = 9'bxx_xx_xxxxx; // []
L[1639] = 9'bxx_xx_xxxxx; // []
L[1640] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1641] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1642] = 9'b11_00_00001; // [PC++]->AH
L[1643] = 9'bxx_xx_xxxxx; // []
L[1644] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1645] = 9'bxx_xx_xxxxx; // []
L[1646] = 9'bxx_xx_xxxxx; // []
L[1647] = 9'bxx_xx_xxxxx; // []
L[1648] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1649] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1650] = 9'b11_00_00001; // [PC++]->AH
L[1651] = 9'bxx_xx_xxxxx; // []
L[1652] = 9'b00_01_00011; // [AX]->T
L[1653] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1654] = 9'b10_01_00101; // T->[AX]
L[1655] = 9'bxx_xx_xxxxx; // []
L[1656] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1657] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1658] = 9'b11_00_00001; // [PC++]->AH
L[1659] = 9'bxx_xx_xxxxx; // []
L[1660] = 9'b00_01_00011; // [AX]->T
L[1661] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1662] = 9'b10_01_00101; // T->[AX]
L[1663] = 9'bxx_xx_xxxxx; // []
L[1664] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1665] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1666] = 9'b11_00_10001; // PC+T->PC
L[1667] = 9'bxx_xx_xxxxx; // []
L[1668] = 9'b10_00_00011; // ['NO-OP', '']
L[1669] = 9'bxx_xx_xxxxx; // []
L[1670] = 9'bxx_xx_xxxxx; // []
L[1671] = 9'bxx_xx_xxxxx; // []
L[1672] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1673] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1674] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1675] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[1676] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1677] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1678] = 9'bxx_xx_xxxxx; // []
L[1679] = 9'bxx_xx_xxxxx; // []
L[1680] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1681] = 9'b10_00_00011; // ['NO-OP', '']
L[1682] = 9'bxx_xx_xxxxx; // []
L[1683] = 9'bxx_xx_xxxxx; // []
L[1684] = 9'bxx_xx_xxxxx; // []
L[1685] = 9'bxx_xx_xxxxx; // []
L[1686] = 9'bxx_xx_xxxxx; // []
L[1687] = 9'bxx_xx_xxxxx; // []
L[1688] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1689] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1690] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1691] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[1692] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1693] = 9'b00_01_00011; // [AX]->T
L[1694] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1695] = 9'b10_01_00101; // T->[AX]
L[1696] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1697] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1698] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1699] = 9'bxx_xx_xxxxx; // []
L[1700] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[1701] = 9'bxx_xx_xxxxx; // []
L[1702] = 9'bxx_xx_xxxxx; // []
L[1703] = 9'bxx_xx_xxxxx; // []
L[1704] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1705] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1706] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1707] = 9'bxx_xx_xxxxx; // []
L[1708] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1709] = 9'bxx_xx_xxxxx; // []
L[1710] = 9'bxx_xx_xxxxx; // []
L[1711] = 9'bxx_xx_xxxxx; // []
L[1712] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1713] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1714] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1715] = 9'bxx_xx_xxxxx; // []
L[1716] = 9'b00_01_00011; // [AX]->T
L[1717] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1718] = 9'b10_01_00101; // T->[AX]
L[1719] = 9'bxx_xx_xxxxx; // []
L[1720] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1721] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1722] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1723] = 9'bxx_xx_xxxxx; // []
L[1724] = 9'b00_01_00011; // [AX]->T
L[1725] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1726] = 9'b10_01_00101; // T->[AX]
L[1727] = 9'bxx_xx_xxxxx; // []
L[1728] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1729] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1730] = 9'bxx_xx_xxxxx; // []
L[1731] = 9'bxx_xx_xxxxx; // []
L[1732] = 9'bxx_xx_xxxxx; // []
L[1733] = 9'bxx_xx_xxxxx; // []
L[1734] = 9'bxx_xx_xxxxx; // []
L[1735] = 9'bxx_xx_xxxxx; // []
L[1736] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1737] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1738] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1739] = 9'bxx_xx_xxxxx; // []
L[1740] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1741] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1742] = 9'bxx_xx_xxxxx; // []
L[1743] = 9'bxx_xx_xxxxx; // []
L[1744] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1745] = 9'b10_00_00011; // ['NO-OP', '']
L[1746] = 9'bxx_xx_xxxxx; // []
L[1747] = 9'bxx_xx_xxxxx; // []
L[1748] = 9'bxx_xx_xxxxx; // []
L[1749] = 9'bxx_xx_xxxxx; // []
L[1750] = 9'bxx_xx_xxxxx; // []
L[1751] = 9'bxx_xx_xxxxx; // []
L[1752] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1753] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1754] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1755] = 9'bxx_xx_xxxxx; // []
L[1756] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1757] = 9'b00_01_00011; // [AX]->T
L[1758] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1759] = 9'b10_01_00101; // T->[AX]
L[1760] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1761] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1762] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1763] = 9'bxx_xx_xxxxx; // []
L[1764] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1765] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[1766] = 9'bxx_xx_xxxxx; // []
L[1767] = 9'bxx_xx_xxxxx; // []
L[1768] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1769] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1770] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1771] = 9'bxx_xx_xxxxx; // []
L[1772] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1773] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1774] = 9'bxx_xx_xxxxx; // []
L[1775] = 9'bxx_xx_xxxxx; // []
L[1776] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1777] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1778] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1779] = 9'bxx_xx_xxxxx; // []
L[1780] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1781] = 9'b00_01_00011; // [AX]->T
L[1782] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1783] = 9'b10_01_00101; // T->[AX]
L[1784] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1785] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1786] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1787] = 9'bxx_xx_xxxxx; // []
L[1788] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1789] = 9'b00_01_00011; // [AX]->T
L[1790] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1791] = 9'b10_01_00101; // T->[AX]
L[1792] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1793] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1794] = 9'bxx_xx_xxxxx; // []
L[1795] = 9'bxx_xx_xxxxx; // []
L[1796] = 9'bxx_xx_xxxxx; // []
L[1797] = 9'bxx_xx_xxxxx; // []
L[1798] = 9'bxx_xx_xxxxx; // []
L[1799] = 9'bxx_xx_xxxxx; // []
L[1800] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1801] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1802] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1803] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1804] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1805] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1806] = 9'bxx_xx_xxxxx; // []
L[1807] = 9'bxx_xx_xxxxx; // []
L[1808] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1809] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1810] = 9'bxx_xx_xxxxx; // []
L[1811] = 9'bxx_xx_xxxxx; // []
L[1812] = 9'bxx_xx_xxxxx; // []
L[1813] = 9'bxx_xx_xxxxx; // []
L[1814] = 9'bxx_xx_xxxxx; // []
L[1815] = 9'bxx_xx_xxxxx; // []
L[1816] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1817] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1818] = 9'b00_01_00111; // [AX]->?,AL+X->AL
L[1819] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1820] = 9'b00_01_01011; // [AX]->AH,T->AL
L[1821] = 9'b00_01_00011; // [AX]->T
L[1822] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1823] = 9'b10_01_00101; // T->[AX]
L[1824] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1825] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1826] = 9'bxx_xx_xxxxx; // []
L[1827] = 9'bxx_xx_xxxxx; // []
L[1828] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1829] = 9'bxx_xx_xxxxx; // []
L[1830] = 9'bxx_xx_xxxxx; // []
L[1831] = 9'bxx_xx_xxxxx; // []
L[1832] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1833] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1834] = 9'bxx_xx_xxxxx; // []
L[1835] = 9'bxx_xx_xxxxx; // []
L[1836] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1837] = 9'bxx_xx_xxxxx; // []
L[1838] = 9'bxx_xx_xxxxx; // []
L[1839] = 9'bxx_xx_xxxxx; // []
L[1840] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1841] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1842] = 9'bxx_xx_xxxxx; // []
L[1843] = 9'bxx_xx_xxxxx; // []
L[1844] = 9'b00_01_00011; // [AX]->T
L[1845] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1846] = 9'b10_01_00101; // T->[AX]
L[1847] = 9'bxx_xx_xxxxx; // []
L[1848] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1849] = 9'b11_00_00000; // ['[PC++]->AL', '[PC++]->T']
L[1850] = 9'bxx_xx_xxxxx; // []
L[1851] = 9'bxx_xx_xxxxx; // []
L[1852] = 9'b00_01_00011; // [AX]->T
L[1853] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1854] = 9'b10_01_00101; // T->[AX]
L[1855] = 9'bxx_xx_xxxxx; // []
L[1856] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1857] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1858] = 9'bxx_xx_xxxxx; // []
L[1859] = 9'bxx_xx_xxxxx; // []
L[1860] = 9'bxx_xx_xxxxx; // []
L[1861] = 9'bxx_xx_xxxxx; // []
L[1862] = 9'bxx_xx_xxxxx; // []
L[1863] = 9'bxx_xx_xxxxx; // []
L[1864] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1865] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1866] = 9'bxx_xx_xxxxx; // []
L[1867] = 9'bxx_xx_xxxxx; // []
L[1868] = 9'bxx_xx_xxxxx; // []
L[1869] = 9'bxx_xx_xxxxx; // []
L[1870] = 9'bxx_xx_xxxxx; // []
L[1871] = 9'bxx_xx_xxxxx; // []
L[1872] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1873] = 9'b10_00_00011; // ['NO-OP', '']
L[1874] = 9'bxx_xx_xxxxx; // []
L[1875] = 9'bxx_xx_xxxxx; // []
L[1876] = 9'bxx_xx_xxxxx; // []
L[1877] = 9'bxx_xx_xxxxx; // []
L[1878] = 9'bxx_xx_xxxxx; // []
L[1879] = 9'bxx_xx_xxxxx; // []
L[1880] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1881] = 9'b10_00_01101; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
L[1882] = 9'bxx_xx_xxxxx; // []
L[1883] = 9'bxx_xx_xxxxx; // []
L[1884] = 9'bxx_xx_xxxxx; // []
L[1885] = 9'bxx_xx_xxxxx; // []
L[1886] = 9'bxx_xx_xxxxx; // []
L[1887] = 9'bxx_xx_xxxxx; // []
L[1888] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1889] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1890] = 9'b11_00_00001; // [PC++]->AH
L[1891] = 9'bxx_xx_xxxxx; // []
L[1892] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1893] = 9'bxx_xx_xxxxx; // []
L[1894] = 9'bxx_xx_xxxxx; // []
L[1895] = 9'bxx_xx_xxxxx; // []
L[1896] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1897] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1898] = 9'b11_00_00001; // [PC++]->AH
L[1899] = 9'bxx_xx_xxxxx; // []
L[1900] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1901] = 9'bxx_xx_xxxxx; // []
L[1902] = 9'bxx_xx_xxxxx; // []
L[1903] = 9'bxx_xx_xxxxx; // []
L[1904] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1905] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1906] = 9'b11_00_00001; // [PC++]->AH
L[1907] = 9'bxx_xx_xxxxx; // []
L[1908] = 9'b00_01_00011; // [AX]->T
L[1909] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1910] = 9'b10_01_00101; // T->[AX]
L[1911] = 9'bxx_xx_xxxxx; // []
L[1912] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1913] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1914] = 9'b11_00_00001; // [PC++]->AH
L[1915] = 9'bxx_xx_xxxxx; // []
L[1916] = 9'b00_01_00011; // [AX]->T
L[1917] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1918] = 9'b10_01_00101; // T->[AX]
L[1919] = 9'bxx_xx_xxxxx; // []
L[1920] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1921] = 9'b10_00_00000; // ['[PC++]->?', 'PC+1->PC', '']
L[1922] = 9'b11_00_10001; // PC+T->PC
L[1923] = 9'bxx_xx_xxxxx; // []
L[1924] = 9'b10_00_00011; // ['NO-OP', '']
L[1925] = 9'bxx_xx_xxxxx; // []
L[1926] = 9'bxx_xx_xxxxx; // []
L[1927] = 9'bxx_xx_xxxxx; // []
L[1928] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1929] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1930] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1931] = 9'b01_01_01100; // [AX]->AH,T+Y->AL
L[1932] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1933] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1934] = 9'bxx_xx_xxxxx; // []
L[1935] = 9'bxx_xx_xxxxx; // []
L[1936] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1937] = 9'b10_00_00011; // ['NO-OP', '']
L[1938] = 9'bxx_xx_xxxxx; // []
L[1939] = 9'bxx_xx_xxxxx; // []
L[1940] = 9'bxx_xx_xxxxx; // []
L[1941] = 9'bxx_xx_xxxxx; // []
L[1942] = 9'bxx_xx_xxxxx; // []
L[1943] = 9'bxx_xx_xxxxx; // []
L[1944] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1945] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1946] = 9'b00_01_01010; // [AX]->T,AL+1->AL
L[1947] = 9'b00_01_01100; // [AX]->AH,T+Y->AL
L[1948] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1949] = 9'b00_01_00011; // [AX]->T
L[1950] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1951] = 9'b10_01_00101; // T->[AX]
L[1952] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1953] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1954] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1955] = 9'bxx_xx_xxxxx; // []
L[1956] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[1957] = 9'bxx_xx_xxxxx; // []
L[1958] = 9'bxx_xx_xxxxx; // []
L[1959] = 9'bxx_xx_xxxxx; // []
L[1960] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1961] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1962] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1963] = 9'bxx_xx_xxxxx; // []
L[1964] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1965] = 9'bxx_xx_xxxxx; // []
L[1966] = 9'bxx_xx_xxxxx; // []
L[1967] = 9'bxx_xx_xxxxx; // []
L[1968] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1969] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1970] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1971] = 9'bxx_xx_xxxxx; // []
L[1972] = 9'b00_01_00011; // [AX]->T
L[1973] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1974] = 9'b10_01_00101; // T->[AX]
L[1975] = 9'bxx_xx_xxxxx; // []
L[1976] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1977] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1978] = 9'b11_01_00111; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL']
L[1979] = 9'bxx_xx_xxxxx; // []
L[1980] = 9'b00_01_00011; // [AX]->T
L[1981] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[1982] = 9'b10_01_00101; // T->[AX]
L[1983] = 9'bxx_xx_xxxxx; // []
L[1984] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1985] = 9'b10_00_00010; // ['[PC]->,ALU()->A', 'Setappropriateflags', 'ALU()->X,Y', 'ALU()->A']
L[1986] = 9'bxx_xx_xxxxx; // []
L[1987] = 9'bxx_xx_xxxxx; // []
L[1988] = 9'bxx_xx_xxxxx; // []
L[1989] = 9'bxx_xx_xxxxx; // []
L[1990] = 9'bxx_xx_xxxxx; // []
L[1991] = 9'bxx_xx_xxxxx; // []
L[1992] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1993] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[1994] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[1995] = 9'bxx_xx_xxxxx; // []
L[1996] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[1997] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[1998] = 9'bxx_xx_xxxxx; // []
L[1999] = 9'bxx_xx_xxxxx; // []
L[2000] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2001] = 9'b10_00_00011; // ['NO-OP', '']
L[2002] = 9'bxx_xx_xxxxx; // []
L[2003] = 9'bxx_xx_xxxxx; // []
L[2004] = 9'bxx_xx_xxxxx; // []
L[2005] = 9'bxx_xx_xxxxx; // []
L[2006] = 9'bxx_xx_xxxxx; // []
L[2007] = 9'bxx_xx_xxxxx; // []
L[2008] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2009] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2010] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[2011] = 9'bxx_xx_xxxxx; // []
L[2012] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[2013] = 9'b00_01_00011; // [AX]->T
L[2014] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[2015] = 9'b10_01_00101; // T->[AX]
L[2016] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2017] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2018] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[2019] = 9'bxx_xx_xxxxx; // []
L[2020] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[2021] = 9'b10_01_00011; // ['ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->']
L[2022] = 9'bxx_xx_xxxxx; // []
L[2023] = 9'bxx_xx_xxxxx; // []
L[2024] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2025] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2026] = 9'b01_00_01000; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[2027] = 9'bxx_xx_xxxxx; // []
L[2028] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[2029] = 9'b10_01_00010; // ['ALU([AX])->A', 'ALU([AX])->?']
L[2030] = 9'bxx_xx_xxxxx; // []
L[2031] = 9'bxx_xx_xxxxx; // []
L[2032] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2033] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2034] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[2035] = 9'bxx_xx_xxxxx; // []
L[2036] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[2037] = 9'b00_01_00011; // [AX]->T
L[2038] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[2039] = 9'b10_01_00101; // T->[AX]
L[2040] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2041] = 9'b00_00_00000; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T']
L[2042] = 9'b11_00_01000; // ['[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+Y->AL']
L[2043] = 9'bxx_xx_xxxxx; // []
L[2044] = 9'b00_01_01001; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
L[2045] = 9'b00_01_00011; // [AX]->T
L[2046] = 9'b00_01_00100; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
L[2047] = 9'b10_01_00101; // T->[AX]
  end  
  always @(posedge clk) if (reset) begin
    M <= 0; // Stupid XILINX inferral only allows 0 as reset value.
  end else if (ce) begin
    M <= L[{IR, State}];    
  end
endmodule

module MicroCodeTable(input clk, input ce, input reset, input [7:0] IR, input [2:0] State, output [37:0] Mout);
  wire [8:0] M;
  MicroCodeTableInner inner(clk, ce, reset, IR, State, M);
  reg [14:0] A[0:31];
  reg [18:0] B[0:255];
  initial begin
A[0] = 15'b_10__0_10101_0xx_01_00; // ['[PC++]', '[PC++]->AL', '[PC++]->?', '[PC++]->T', 'PC+1->PC', '']
A[1] = 15'b_xx__0_0xx11_0xx_01_00; // [PC++]->AH
A[2] = 15'b_xx__1_00000_0xx_00_00; // ['ALU([AX])->A', 'ALU([AX])->?', '[PC]->,ALU()->A', 'Setappropriateflags', 'ALU([SP])->A', '[SP]->P', 'ALU()->X,Y', 'ALU()->A']
A[3] = 15'b_10__0_00000_0xx_00_00; // ['[AX]->T', 'ALU([AX])->?', 'ALU([AX])->A', 'ALU([AX])->', '[PC]->', 'NO-OP', '[VECT]->T', '']
A[4] = 15'b_11__1_00000_100_00_00; // ['T->[AX],ALU(T)->T', 'T->[AX],ALU(T)->T:A']
A[5] = 15'b_xx__0_xxxxx_100_00_00; // T->[AX]
A[6] = 15'b_xx__0_00000_101_00_00; // ALU()->[AX]
A[7] = 15'b_0x__0_10000_0xx_00_00; // ['[AX]->?,AL+X->AL', '[AX]->?,AL+X/Y->AL', 'KEEP_AC']
A[8] = 15'b_xx__0_10011_0xx_01_00; // ['[PC++]->AH,AL+X/Y->AL', '[PC++]->AH,AL+X->AL', '[PC++]->AH,AL+Y->AL']
A[9] = 15'b_10__0_0xx10_0xx_00_00; // ['[AX]->?,AH+FIX->AH', '[AX]->T,AH+FIX->AH']
A[10] = 15'b_10__0_11000_0xx_00_00; // [AX]->T,AL+1->AL
A[11] = 15'b_xx__0_11111_0xx_00_00; // [AX]->AH,T->AL
A[12] = 15'b_xx__0_10011_0xx_00_00; // [AX]->AH,T+Y->AL
A[13] = 15'b_xx__1_xxxxx_0xx_01_00; // ['ALU([PC++])->A', 'ALU([PC++])->?', 'ALU([PC++])->Reg']
A[14] = 15'b_xx__0_xxxxx_101_00_11; // ALU(A)->[SP--]
A[15] = 15'b_xx__0_xxxxx_110_00_11; // P->[SP--]
A[16] = 15'b_10__0_xxxxx_0xx_00_10; // ['SP++', 'SP+1->SP', '[SP]->T,SP+1->SP']
A[17] = 15'b_xx__0_xxxxx_0xx_11_00; // PC+T->PC
A[18] = 15'b_xx__0_xxxxx_0xx_00_01; // X->S
A[19] = 15'b_xx__0_xxxxx_0xx_10_00; // ['[PC]:T->PC', '[AX]:T->PC', '[VECT]:T->PC', '[SP]:T->PC']
A[20] = 15'b_0x__0_xxxxx_111_00_11; // ['PCH->[SP--]', 'PCL->[SP--],', 'PCH->[SP--](KEEPAC)', 'PCL->[SP--](KEEPAC)']
A[21] = 15'b_xx__1_xxxxx_110_00_11; // P->[SP--]
A[22] = 15'b_xx__1_xxxxx_0xx_00_10; // [SP]->P,SP+1->SP
A[23] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[24] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[25] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[26] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[27] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[28] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[29] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[30] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
A[31] = 15'b_xx__x_xxxxx_xxx_xx_xx; // []
B[0] = 19'bxxxxxxxxxx0_000_00_010;
B[32] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[64] = 19'bxxxxxxxxxx0_000_00_100;
B[96] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[128] = 19'b011010x10x0_xxx_00_xxx;
B[160] = 19'bxx0010x10x0_100_00_001;
B[192] = 19'b010010x1100_000_00_001;
B[224] = 19'b100010x1100_000_00_001;
B[1] = 19'b000010x0001_010_00_001;
B[33] = 19'b000010x0011_010_00_001;
B[65] = 19'b000010x0101_010_00_001;
B[97] = 19'b000010x0111_010_00_001;
B[129] = 19'b001010x10x1_xxx_00_xxx;
B[161] = 19'bxx0010x10x1_010_00_001;
B[193] = 19'b000010x1101_000_00_001;
B[225] = 19'b000010x1111_010_00_001;
B[2] = 19'bxx0100010x0_000_00_001;
B[34] = 19'bxx0100110x0_000_00_001;
B[66] = 19'bxx0101010x0_000_00_001;
B[98] = 19'bxx0101110x0_000_00_001;
B[130] = 19'b101010x10x0_xxx_00_xxx;
B[162] = 19'bxx0010x10x0_001_00_001;
B[194] = 19'bxx0111010x0_000_00_001;
B[226] = 19'bxx0111110x0_000_00_001;
B[3] = 19'b00010000001_010_00_001;
B[35] = 19'b00010010011_010_00_001;
B[67] = 19'b00010100101_010_00_001;
B[99] = 19'b00010110111_010_00_001;
B[131] = 19'b111010x10x1_xxx_00_xxx;
B[163] = 19'bxx0010x10x1_011_00_001;
B[195] = 19'b00011101101_000_00_001;
B[227] = 19'b00011111111_010_00_001;
B[4] = 19'b000010x0000_xxx_00_xxx;
B[36] = 19'b000010x0010_000_00_001;
B[68] = 19'b000010x0100_xxx_00_xxx;
B[100] = 19'b000010x0110_xxx_00_xxx;
B[132] = 19'b011010x10x0_xxx_00_xxx;
B[164] = 19'bxx0010x10x0_100_00_001;
B[196] = 19'b010010x1100_000_00_001;
B[228] = 19'b100010x1100_000_00_001;
B[5] = 19'b000010x0001_010_00_001;
B[37] = 19'b000010x0011_010_00_001;
B[69] = 19'b000010x0101_010_00_001;
B[101] = 19'b000010x0111_010_00_001;
B[133] = 19'b001010x10x1_xxx_00_xxx;
B[165] = 19'bxx0010x10x1_010_00_001;
B[197] = 19'b000010x1101_000_00_001;
B[229] = 19'b000010x1111_010_00_001;
B[6] = 19'bxx0100010x0_000_00_001;
B[38] = 19'bxx0100110x0_000_00_001;
B[70] = 19'bxx0101010x0_000_00_001;
B[102] = 19'bxx0101110x0_000_00_001;
B[134] = 19'b101010x10x0_xxx_00_xxx;
B[166] = 19'bxx0010x10x0_001_00_001;
B[198] = 19'bxx0111010x0_000_00_001;
B[230] = 19'bxx0111110x0_000_00_001;
B[7] = 19'b00010000001_010_00_001;
B[39] = 19'b00010010011_010_00_001;
B[71] = 19'b00010100101_010_00_001;
B[103] = 19'b00010110111_010_00_001;
B[135] = 19'b111010x10x1_xxx_00_xxx;
B[167] = 19'bxx0010x10x1_011_00_001;
B[199] = 19'b00011101101_000_00_001;
B[231] = 19'b00011111111_010_00_001;
B[8] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[40] = 19'bxxxxxxxxxx0_000_00_100;
B[72] = 19'b001010x10x0_xxx_00_xxx;
B[104] = 19'bxx0010x10x0_010_00_001;
B[136] = 19'b011011010x0_100_00_001;
B[168] = 19'b001010x10x0_100_00_001;
B[200] = 19'b011011110x0_100_00_001;
B[232] = 19'b101011110x0_001_00_001;
B[9] = 19'b000010x0001_010_00_001;
B[41] = 19'b000010x0011_010_00_001;
B[73] = 19'b000010x0101_010_00_001;
B[105] = 19'b000010x0111_010_00_001;
B[137] = 19'b001010x10x1_xxx_00_xxx;
B[169] = 19'bxx0010x10x1_010_00_001;
B[201] = 19'b000010x1101_000_00_001;
B[233] = 19'b000010x1111_010_00_001;
B[10] = 19'b001000010x0_010_00_001;
B[42] = 19'b001000110x0_010_00_001;
B[74] = 19'b001001010x0_010_00_001;
B[106] = 19'b001001110x0_010_00_001;
B[138] = 19'b101010x10x0_010_00_001;
B[170] = 19'b001010x10x0_001_00_001;
B[202] = 19'b101011010x0_001_00_001;
B[234] = 19'bxx0111110x0_000_00_001;
B[11] = 19'b000010x0001_010_00_001;
B[43] = 19'b000010x0011_010_00_001;
B[75] = 19'b000010x0101_010_00_001;
B[107] = 19'b000010x0111_010_00_001;
B[139] = 19'b111010x10x1_xxx_00_xxx;
B[171] = 19'bxx0010x10x1_011_00_001;
B[203] = 19'b000010x1101_000_00_001;
B[235] = 19'b000010x1111_010_00_001;
B[12] = 19'b000010x0000_xxx_00_xxx;
B[44] = 19'b000010x0010_000_00_001;
B[76] = 19'bxxxxxxxxxx0_000_00_001;
B[108] = 19'bxxxxxxxxxx0_000_00_001;
B[140] = 19'b011010x10x0_xxx_00_xxx;
B[172] = 19'bxx0010x10x0_100_00_001;
B[204] = 19'b010010x1100_000_00_001;
B[236] = 19'b100010x1100_000_00_001;
B[13] = 19'b000010x0001_010_00_001;
B[45] = 19'b000010x0011_010_00_001;
B[77] = 19'b000010x0101_010_00_001;
B[109] = 19'b000010x0111_010_00_001;
B[141] = 19'b001010x10x1_xxx_00_xxx;
B[173] = 19'bxx0010x10x1_010_00_001;
B[205] = 19'b000010x1101_000_00_001;
B[237] = 19'b000010x1111_010_00_001;
B[14] = 19'bxx0100010x0_000_00_001;
B[46] = 19'bxx0100110x0_000_00_001;
B[78] = 19'bxx0101010x0_000_00_001;
B[110] = 19'bxx0101110x0_000_00_001;
B[142] = 19'b101010x10x0_xxx_00_xxx;
B[174] = 19'bxx0010x10x0_001_00_001;
B[206] = 19'bxx0111010x0_000_00_001;
B[238] = 19'bxx0111110x0_000_00_001;
B[15] = 19'b00010000001_010_00_001;
B[47] = 19'b00010010011_010_00_001;
B[79] = 19'b00010100101_010_00_001;
B[111] = 19'b00010110111_010_00_001;
B[143] = 19'b111010x10x1_xxx_00_xxx;
B[175] = 19'bxx0010x10x1_011_00_001;
B[207] = 19'b00011101101_000_00_001;
B[239] = 19'b00011111111_010_00_001;
B[16] = 19'bxxxxxxxxxx0_xxx_11_xxx;
B[48] = 19'bxxxxxxxxxx0_xxx_11_xxx;
B[80] = 19'bxxxxxxxxxx0_xxx_11_xxx;
B[112] = 19'bxxxxxxxxxx0_xxx_11_xxx;
B[144] = 19'b011010x10x0_xxx_11_xxx;
B[176] = 19'bxx0010x10x0_xxx_11_xxx;
B[208] = 19'bxxxxxxxxxx0_xxx_11_xxx;
B[240] = 19'bxxxxxxxxxx0_xxx_11_xxx;
B[17] = 19'b000010x0001_010_11_001;
B[49] = 19'b000010x0011_010_11_001;
B[81] = 19'b000010x0101_010_11_001;
B[113] = 19'b000010x0111_010_11_001;
B[145] = 19'b001010x10x1_xxx_11_xxx;
B[177] = 19'bxx0010x10x1_010_11_001;
B[209] = 19'b000010x1101_000_11_001;
B[241] = 19'b000010x1111_010_11_001;
B[18] = 19'bxx0100010x0_000_11_001;
B[50] = 19'bxx0100110x0_000_11_001;
B[82] = 19'bxx0101010x0_000_11_001;
B[114] = 19'bxx0101110x0_000_11_001;
B[146] = 19'b101010x10x0_xxx_11_xxx;
B[178] = 19'bxx0010x10x0_xxx_11_xxx;
B[210] = 19'bxx0111010x0_000_11_001;
B[242] = 19'bxx0111110x0_000_11_001;
B[19] = 19'b00010000001_010_11_001;
B[51] = 19'b00010010011_010_11_001;
B[83] = 19'b00010100101_010_11_001;
B[115] = 19'b00010110111_010_11_001;
B[147] = 19'b111010x10x1_xxx_11_xxx;
B[179] = 19'bxx0010x10x1_011_11_001;
B[211] = 19'b00011101101_000_11_001;
B[243] = 19'b00011111111_010_11_001;
B[20] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[52] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[84] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[116] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[148] = 19'b011010x10x0_xxx_00_xxx;
B[180] = 19'bxx0010x10x0_100_00_001;
B[212] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[244] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[21] = 19'b000010x0001_010_00_001;
B[53] = 19'b000010x0011_010_00_001;
B[85] = 19'b000010x0101_010_00_001;
B[117] = 19'b000010x0111_010_00_001;
B[149] = 19'b001010x10x1_xxx_00_xxx;
B[181] = 19'bxx0010x10x1_010_00_001;
B[213] = 19'b000010x1101_000_00_001;
B[245] = 19'b000010x1111_010_00_001;
B[22] = 19'bxx0100010x0_000_00_001;
B[54] = 19'bxx0100110x0_000_00_001;
B[86] = 19'bxx0101010x0_000_00_001;
B[118] = 19'bxx0101110x0_000_00_001;
B[150] = 19'b101010x10x0_xxx_10_xxx;
B[182] = 19'bxx0010x10x0_001_10_001;
B[214] = 19'bxx0111010x0_000_00_001;
B[246] = 19'bxx0111110x0_000_00_001;
B[23] = 19'b00010000001_010_00_001;
B[55] = 19'b00010010011_010_00_001;
B[87] = 19'b00010100101_010_00_001;
B[119] = 19'b00010110111_010_00_001;
B[151] = 19'b111010x10x1_xxx_10_xxx;
B[183] = 19'bxx0010x10x1_011_10_001;
B[215] = 19'b00011101101_000_00_001;
B[247] = 19'b00011111111_010_00_001;
B[24] = 19'bxxxxxxxxxx0_000_10_101;
B[56] = 19'bxxxxxxxxxx0_000_10_101;
B[88] = 19'bxxxxxxxxxx0_000_10_110;
B[120] = 19'bxxxxxxxxxx0_000_10_110;
B[152] = 19'b011010x10x0_010_10_001;
B[184] = 19'bxx1110x10x0_000_10_011;
B[216] = 19'bxxxxxxxxxx0_000_10_111;
B[248] = 19'bxxxxxxxxxx0_000_10_111;
B[25] = 19'b000010x0001_010_10_001;
B[57] = 19'b000010x0011_010_10_001;
B[89] = 19'b000010x0101_010_10_001;
B[121] = 19'b000010x0111_010_10_001;
B[153] = 19'b001010x10x1_xxx_10_xxx;
B[185] = 19'bxx0010x10x1_010_10_001;
B[217] = 19'b000010x1101_000_10_001;
B[249] = 19'b000010x1111_010_10_001;
B[26] = 19'bxx0100010x0_000_10_001;
B[58] = 19'bxx0100110x0_000_10_001;
B[90] = 19'bxx0101010x0_000_10_001;
B[122] = 19'bxx0101110x0_000_10_001;
B[154] = 19'b101010x10x0_xxx_10_xxx;
B[186] = 19'bxx1110x10x0_001_10_001;
B[218] = 19'bxx0111010x0_000_10_001;
B[250] = 19'bxx0111110x0_000_10_001;
B[27] = 19'b00010000001_010_10_001;
B[59] = 19'b00010010011_010_10_001;
B[91] = 19'b00010100101_010_10_001;
B[123] = 19'b00010110111_010_10_001;
B[155] = 19'b111010x10x1_xxx_10_xxx;
B[187] = 19'bxx0010x10x1_xxx_10_xxx;
B[219] = 19'b00011101101_000_10_001;
B[251] = 19'b00011111111_010_10_001;
B[28] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[60] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[92] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[124] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[156] = 19'b011010x10x0_xxx_00_xxx;
B[188] = 19'bxx0010x10x0_100_00_001;
B[220] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[252] = 19'bxxxxxxxxxx0_xxx_00_xxx;
B[29] = 19'b000010x0001_010_00_001;
B[61] = 19'b000010x0011_010_00_001;
B[93] = 19'b000010x0101_010_00_001;
B[125] = 19'b000010x0111_010_00_001;
B[157] = 19'b001010x10x1_xxx_00_xxx;
B[189] = 19'bxx0010x10x1_010_00_001;
B[221] = 19'b000010x1101_000_00_001;
B[253] = 19'b000010x1111_010_00_001;
B[30] = 19'bxx0100010x0_000_00_001;
B[62] = 19'bxx0100110x0_000_00_001;
B[94] = 19'bxx0101010x0_000_00_001;
B[126] = 19'bxx0101110x0_000_00_001;
B[158] = 19'b101010x10x0_xxx_10_xxx;
B[190] = 19'bxx0010x10x0_001_10_001;
B[222] = 19'bxx0111010x0_000_00_001;
B[254] = 19'bxx0111110x0_000_00_001;
B[31] = 19'b00010000001_010_00_001;
B[63] = 19'b00010010011_010_00_001;
B[95] = 19'b00010100101_010_00_001;
B[127] = 19'b00010110111_010_00_001;
B[159] = 19'b111010x10x1_xxx_10_xxx;
B[191] = 19'bxx0010x10x1_011_10_001;
B[223] = 19'b00011101101_000_00_001;
B[255] = 19'b00011111111_010_00_001;
  end  
  wire [14:0] R = A[M[4:0]];
  reg [18:0] AluFlags;
  always @(posedge clk) if (reset) begin
    AluFlags <= 0;
  end else if (ce) begin  
    AluFlags <= B[IR];
  end

  assign Mout = {AluFlags,// 19
                 M[8:7],  // NextState // 2
                 R[14:13],// LoadT     // 2
                 R[12],   // FlagCtrl  // 1
                 R[11:7], // AddrCtrl  // 5
                 R[6:4],  // MemWrite  // 3
                 M[6:5],  // AddrBus   // 2
                 R[3:2],  // LoadPC    // 2
                 R[1:0]   // LoadSP    // 2
                 };
endmodule


// Copyright (c) 2012-2013 Ludvig Strigeus
// This program is GPL Licensed. See COPYING for the full license.
 
module LenCtr_Lookup(input [4:0] X, output [7:0] Yout);
reg [6:0] Y;
always @*
begin
  case(X)
  0: Y = 7'h05;
  1: Y = 7'h7F;
  2: Y = 7'h0A;
  3: Y = 7'h01;
  4: Y = 7'h14;
  5: Y = 7'h02;
  6: Y = 7'h28;
  7: Y = 7'h03;
  8: Y = 7'h50;
  9: Y = 7'h04;
  10: Y = 7'h1E;
  11: Y = 7'h05;
  12: Y = 7'h07;
  13: Y = 7'h06;
  14: Y = 7'h0D;
  15: Y = 7'h07;
  16: Y = 7'h06;
  17: Y = 7'h08;
  18: Y = 7'h0C;
  19: Y = 7'h09;
  20: Y = 7'h18;
  21: Y = 7'h0A;
  22: Y = 7'h30;
  23: Y = 7'h0B;
  24: Y = 7'h60;
  25: Y = 7'h0C;
  26: Y = 7'h24;
  27: Y = 7'h0D;
  28: Y = 7'h08;
  29: Y = 7'h0E;
  30: Y = 7'h10;
  31: Y = 7'h0F;
  endcase
end
assign Yout = {Y, 1'b0};
endmodule

module SquareChan(input clk, input ce, input reset, input sq2,
                  input [1:0] Addr,
                  input [7:0] DIN,
                  input MW,
                  input LenCtr_Clock,
                  input Env_Clock,
                  input Enabled,
                  input [7:0] LenCtr_In,
                  output reg [3:0] Sample,
                  output IsNonZero);
reg [7:0] LenCtr;

// Register 1
reg [1:0] Duty;
reg EnvLoop, EnvDisable, EnvDoReset;
reg [3:0] Volume, Envelope, EnvDivider;
wire LenCtrHalt = EnvLoop; // Aliased bit
assign IsNonZero = (LenCtr != 0);
// Register 2
reg SweepEnable, SweepNegate, SweepReset;
reg [2:0] SweepPeriod, SweepDivider, SweepShift;

reg [10:0] Period;
reg [11:0] TimerCtr;
reg [2:0] SeqPos;
wire [10:0] ShiftedPeriod = (Period >> SweepShift);
wire [10:0] PeriodRhs = (SweepNegate ? (~ShiftedPeriod + {10'b0, sq2}) : ShiftedPeriod);
wire [11:0] NewSweepPeriod = Period + PeriodRhs;
wire ValidFreq = Period[10:3] >= 8 && (SweepNegate || !NewSweepPeriod[11]);

always @(posedge clk) if (reset) begin
    LenCtr <= 0;
    Duty <= 0;
    EnvDoReset <= 0;
    EnvLoop <= 0;
    EnvDisable <= 0;
    Volume <= 0;
    Envelope <= 0;
    EnvDivider <= 0;
    SweepEnable <= 0;
    SweepNegate <= 0;
    SweepReset <= 0;
    SweepPeriod <= 0;
    SweepDivider <= 0;
    SweepShift <= 0;    
    Period <= 0;
    TimerCtr <= 0;
    SeqPos <= 0;
  end else if (ce) begin
  // Check if writing to the regs of this channel
  // NOTE: This needs to be done before the clocking below.
  if (MW) begin
    case(Addr)
    0: begin
//      if (sq2) $write("SQ0: Duty=%d, EnvLoop=%d, EnvDisable=%d, Volume=%d\n", DIN[7:6], DIN[5], DIN[4], DIN[3:0]);
      Duty <= DIN[7:6];
      EnvLoop <= DIN[5];
      EnvDisable <= DIN[4];
      Volume <= DIN[3:0];
    end
    1: begin
//      if (sq2) $write("SQ1: SweepEnable=%d, SweepPeriod=%d, SweepNegate=%d, SweepShift=%d, DIN=%X\n", DIN[7], DIN[6:4], DIN[3], DIN[2:0], DIN);
      SweepEnable <= DIN[7];
      SweepPeriod <= DIN[6:4];
      SweepNegate <= DIN[3];
      SweepShift <= DIN[2:0];
      SweepReset <= 1;
    end
    2: begin
//      if (sq2) $write("SQ2: Period=%d. DIN=%X\n", DIN, DIN);
      Period[7:0] <= DIN;
    end
    3: begin
      // Upper bits of the period
//      if (sq2) $write("SQ3: PeriodUpper=%d LenCtr=%x DIN=%X\n", DIN[2:0], LenCtr_In, DIN);
      Period[10:8] <= DIN[2:0];
      LenCtr <= LenCtr_In;
      EnvDoReset <= 1;
      SeqPos <= 0;
    end
    endcase
  end

  
  // Count down the square timer...
  if (TimerCtr == 0) begin
    // Timer was clocked
    TimerCtr <= {Period, 1'b0};
    SeqPos <= SeqPos - 1;
  end else begin
    TimerCtr <= TimerCtr - 1;
  end

  // Clock the length counter?
  if (LenCtr_Clock && LenCtr != 0 && !LenCtrHalt) begin
    LenCtr <= LenCtr - 1;
  end
  
  // Clock the sweep unit?
  if (LenCtr_Clock) begin
    if (SweepDivider == 0) begin
      SweepDivider <= SweepPeriod;
      if (SweepEnable && SweepShift != 0 && ValidFreq)
        Period <= NewSweepPeriod[10:0];
    end else begin
      SweepDivider <= SweepDivider - 1;
    end
    if (SweepReset)
      SweepDivider <= SweepPeriod;
    SweepReset <= 0;
  end
  
  // Clock the envelope generator?
  if (Env_Clock) begin
    if (EnvDoReset) begin
      EnvDivider <= Volume;
      Envelope <= 15;
      EnvDoReset <= 0;
    end else if (EnvDivider == 0) begin
      EnvDivider <= Volume;
      if (Envelope != 0 || EnvLoop)
        Envelope <= Envelope - 1;
    end else begin
      EnvDivider <= EnvDivider - 1;
    end
  end
 
  // Length counter forced to zero if disabled.
  if (!Enabled)
    LenCtr <= 0;
end

reg DutyEnabled;  
always @* begin
  // Determine if the duty is enabled or not
  case (Duty)
  0: DutyEnabled = (SeqPos == 7);
  1: DutyEnabled = (SeqPos >= 6);
  2: DutyEnabled = (SeqPos >= 4);
  3: DutyEnabled = (SeqPos < 6);
  endcase

  // Compute the output
  if (LenCtr == 0 || !ValidFreq || !DutyEnabled)
    Sample = 0;
  else
    Sample = EnvDisable ? Volume : Envelope;
end
endmodule



module TriangleChan(input clk, input ce, input reset,
                    input [1:0] Addr,
                    input [7:0] DIN,
                    input MW,
                    input LenCtr_Clock,
                    input LinCtr_Clock,
                    input Enabled,
                    input [7:0] LenCtr_In,
                    output [3:0] Sample,
                    output IsNonZero);
  //
  reg [10:0] Period, TimerCtr;
  reg [4:0] SeqPos;
  //
  // Linear counter state
  reg [6:0] LinCtrPeriod, LinCtr;
  reg LinCtrl, LinHalt;
  wire LinCtrZero = (LinCtr == 0);
  //
  // Length counter state
  reg [7:0] LenCtr;
  wire LenCtrHalt = LinCtrl; // Aliased bit
  wire LenCtrZero = (LenCtr == 0);
  assign IsNonZero = !LenCtrZero;
  //
  always @(posedge clk) if (reset) begin
    Period <= 0;
    TimerCtr <= 0;
    SeqPos <= 0;
    LinCtrPeriod <= 0;
    LinCtr <= 0;
    LinCtrl <= 0;
    LinHalt <= 0;
    LenCtr <= 0;
  end else if (ce) begin
    // Check if writing to the regs of this channel 
    if (MW) begin
      case (Addr)
      0: begin
        LinCtrl <= DIN[7];
        LinCtrPeriod <= DIN[6:0];
      end
      2: begin
        Period[7:0] <= DIN;
      end
      3: begin
        Period[10:8] <= DIN[2:0];
        LenCtr <= LenCtr_In;
        LinHalt <= 1;
      end
      endcase
    end

    // Count down the period timer...
    if (TimerCtr == 0) begin
      TimerCtr <= Period;
    end else begin
      TimerCtr <= TimerCtr - 1;
    end
    //
    // Clock the length counter?
    if (LenCtr_Clock && !LenCtrZero && !LenCtrHalt) begin
      LenCtr <= LenCtr - 1;
    end
    //
    // Clock the linear counter?
    if (LinCtr_Clock) begin
      if (LinHalt)
        LinCtr <= LinCtrPeriod;
      else if (!LinCtrZero)
        LinCtr <= LinCtr - 1;
      if (!LinCtrl)
        LinHalt <= 0;
    end
    //
    // Length counter forced to zero if disabled.
    if (!Enabled)
      LenCtr <= 0;
      //
    // Clock the sequencer position
    if (TimerCtr == 0 && !LenCtrZero && !LinCtrZero)
      SeqPos <= SeqPos + 1;
  end
  // Generate the output
  assign Sample = SeqPos[3:0] ^ {4{~SeqPos[4]}};
  //
endmodule


module NoiseChan(input clk, input ce, input reset,
                 input [1:0] Addr,
                 input [7:0] DIN,
                 input MW,
                 input LenCtr_Clock,
                 input Env_Clock,
                 input Enabled,
                 input [7:0] LenCtr_In,
                 output [3:0] Sample,
                 output IsNonZero);
  //
  // Envelope volume
  reg EnvLoop, EnvDisable, EnvDoReset;
  reg [3:0] Volume, Envelope, EnvDivider;
  // Length counter
  wire LenCtrHalt = EnvLoop; // Aliased bit
  reg [7:0] LenCtr;
  //
  reg ShortMode;
  reg [14:0] Shift = 1;
  
  assign IsNonZero = (LenCtr != 0);
  //
  // Period stuff
  reg [3:0] Period;
  reg [11:0] NoisePeriod, TimerCtr;
  always @* begin
    case (Period)
    0: NoisePeriod = 12'h004;
    1: NoisePeriod = 12'h008;
    2: NoisePeriod = 12'h010;
    3: NoisePeriod = 12'h020;
    4: NoisePeriod = 12'h040;
    5: NoisePeriod = 12'h060;
    6: NoisePeriod = 12'h080;
    7: NoisePeriod = 12'h0A0;
    8: NoisePeriod = 12'h0CA;
    9: NoisePeriod = 12'h0FE;
    10: NoisePeriod = 12'h17C;
    11: NoisePeriod = 12'h1FC;
    12: NoisePeriod = 12'h2FA;
    13: NoisePeriod = 12'h3F8;
    14: NoisePeriod = 12'h7F2;
    15: NoisePeriod = 12'hFE4;  
    endcase
  end
  //
  always @(posedge clk) if (reset) begin
    EnvLoop <= 0;
    EnvDisable <= 0;
    EnvDoReset <= 0;
    Volume <= 0;
    Envelope <= 0;
    EnvDivider <= 0;
    LenCtr <= 0;
    ShortMode <= 0;
    Shift <= 1;
    Period <= 0;
    TimerCtr <= 0;
  end else if (ce) begin
    // Check if writing to the regs of this channel 
    if (MW) begin
      case (Addr)
      0: begin
        EnvLoop <= DIN[5];
        EnvDisable <= DIN[4];
        Volume <= DIN[3:0];
      end
      2: begin
        ShortMode <= DIN[7];
        Period <= DIN[3:0];
      end
      3: begin
        LenCtr <= LenCtr_In;
        EnvDoReset <= 1;
      end
      endcase
    end
    // Count down the period timer...
    if (TimerCtr == 0) begin
      TimerCtr <= NoisePeriod;
      // Clock the shift register. Use either 
      // bit 1 or 6 as the tap.
      Shift <= { 
        Shift[0] ^ (ShortMode ? Shift[6] : Shift[1]), 
        Shift[14:1]}; 
    end else begin
      TimerCtr <= TimerCtr - 1;
    end
    // Clock the length counter?
    if (LenCtr_Clock && LenCtr != 0 && !LenCtrHalt) begin
      LenCtr <= LenCtr - 1;
    end
    // Clock the envelope generator?
    if (Env_Clock) begin
      if (EnvDoReset) begin
        EnvDivider <= Volume;
        Envelope <= 15;
        EnvDoReset <= 0;
      end else if (EnvDivider == 0) begin
        EnvDivider <= Volume;
        if (Envelope != 0)
          Envelope <= Envelope - 1;
        else if (EnvLoop)
          Envelope <= 15;
      end else
        EnvDivider <= EnvDivider - 1;
    end
    if (!Enabled)
      LenCtr <= 0;
  end
  // Produce the output signal
  assign Sample = 
    (LenCtr == 0 || Shift[0]) ?
      0 : 
      (EnvDisable ? Volume : Envelope);
endmodule

module DmcChan(input clk, input ce, input reset,
               input odd_or_even,
               input [2:0] Addr,
               input [7:0] DIN,
               input MW,
               output [6:0] Sample,
               output DmaReq,          // 1 when DMC wants DMA
               input DmaAck,           // 1 when DMC byte is on DmcData. DmcDmaRequested should go low.
               output [15:0] DmaAddr,  // Address DMC wants to read
               input [7:0] DmaData,    // Input data to DMC from memory.
               output Irq,
               output IsDmcActive);
  reg IrqEnable;
  reg IrqActive;
  reg Loop;                 // Looping enabled
  reg [3:0] Freq;           // Current value of frequency register
  reg [6:0] Dac = 0;        // Current value of DAC
  reg [7:0] SampleAddress;  // Base address of sample
  reg [7:0] SampleLen;      // Length of sample
  reg [7:0] ShiftReg;       // Shift register
  reg [8:0] Cycles;         // Down counter, is the period
  reg [14:0] Address;        // 15 bits current address, 0x8000-0xffff
  reg [11:0] BytesLeft;      // 12 bits bytes left counter 0 - 4081.
  reg [2:0] BitsUsed;        // Number of bits left in the SampleBuffer.
  reg [7:0] SampleBuffer;    // Next value to be loaded into shift reg
  reg HasSampleBuffer;       // Sample buffer is nonempty
  reg HasShiftReg;           // Shift reg is non empty
  reg [8:0] NewPeriod[0:15];
  reg DmcEnabled;
  reg [1:0] ActivationDelay;
  assign DmaAddr = {1'b1, Address};
  assign Sample = Dac;
  assign Irq = IrqActive;
  assign IsDmcActive = DmcEnabled;
  
  assign DmaReq = !HasSampleBuffer && DmcEnabled && !ActivationDelay[0];
    
  initial begin
    NewPeriod[0] = 428;
    NewPeriod[1] = 380;
    NewPeriod[2] = 340;
    NewPeriod[3] = 320;
    NewPeriod[4] = 286;
    NewPeriod[5] = 254;
    NewPeriod[6] = 226;
    NewPeriod[7] = 214;
    NewPeriod[8] = 190;
    NewPeriod[9] = 160;
    NewPeriod[10] = 142;
    NewPeriod[11] = 128;
    NewPeriod[12] = 106;
    NewPeriod[13] = 84;
    NewPeriod[14] = 72;
    NewPeriod[15] = 54;
  end
  // Shift register initially loaded with 07
  always @(posedge clk) begin
    if (reset) begin
      IrqEnable <= 0;
      IrqActive <= 0;
      Loop <= 0;
      Freq <= 0;
      Dac <= 0;
      SampleAddress <= 0;
      SampleLen <= 0;
      ShiftReg <= 8'hff;
      Cycles <= 439;
      Address <= 0;
      BytesLeft <= 0;
      BitsUsed <= 0;
      SampleBuffer <= 0;
      HasSampleBuffer <= 0;
      HasShiftReg <= 0;
      DmcEnabled <= 0;
      ActivationDelay <= 0;
    end else if (ce) begin
      if (ActivationDelay == 3 && !odd_or_even) ActivationDelay <= 1;
      if (ActivationDelay == 1) ActivationDelay <= 0;
      
      if (MW) begin
        case (Addr)
        0: begin  // $4010   il-- ffff   IRQ enable, loop, frequency index
            IrqEnable <= DIN[7];
            Loop <= DIN[6];
            Freq <= DIN[3:0];
            if (!DIN[7]) IrqActive <= 0;
          end
        1: begin  // $4011   -ddd dddd   DAC
            // This will get missed if the Dac <= far below runs, that is by design.
            Dac <= DIN[6:0];
          end
        2: begin  // $4012   aaaa aaaa   sample address
            SampleAddress <= DIN[7:0];
          end
        3: begin  // $4013   llll llll   sample length
            SampleLen <= DIN[7:0];
          end
        5: begin // $4015 write ---D NT21  Enable DMC (D)
            IrqActive <= 0;
            DmcEnabled <= DIN[4];
            // If the DMC bit is set, the DMC sample will be restarted only if not already active.
            if (DIN[4] && !DmcEnabled) begin
              Address <= {1'b1, SampleAddress, 6'b000000};
              BytesLeft <= {SampleLen, 4'b0000};
              ActivationDelay <= 3;
            end
          end
        endcase
      end

      Cycles <= Cycles - 1;
      if (Cycles == 1) begin
        Cycles <= NewPeriod[Freq];
        if (HasShiftReg) begin
          if (ShiftReg[0]) begin
            Dac[6:1] <= (Dac[6:1] != 6'b111111) ? Dac[6:1] + 6'b000001 : Dac[6:1];
          end else begin
            Dac[6:1] <= (Dac[6:1] != 6'b000000) ? Dac[6:1] + 6'b111111 : Dac[6:1];
          end
        end
        ShiftReg <= {1'b0, ShiftReg[7:1]};
        BitsUsed <= BitsUsed + 1;
        if (BitsUsed == 7) begin
          HasShiftReg <= HasSampleBuffer;
          ShiftReg <= SampleBuffer;
          HasSampleBuffer <= 0;
        end
      end
      
      // Acknowledge DMA?
      if (DmaAck) begin
        Address <= Address + 1;
        BytesLeft <= BytesLeft - 1;
        HasSampleBuffer <= 1;
        SampleBuffer <= DmaData;
        if (BytesLeft == 0) begin
          Address <= {1'b1, SampleAddress, 6'b000000};
          BytesLeft <= {SampleLen, 4'b0000};
          DmcEnabled <= Loop;
          if (!Loop && IrqEnable)
            IrqActive <= 1;
        end
      end      
    end
  end
endmodule

module ApuLookupTable(input clk, input [7:0] in_a, input [7:0] in_b, output [15:0] out);
  reg [15:0] lookup[0:511];
  reg [15:0] tmp_a, tmp_b;
  initial begin
    lookup[  0] =     0; lookup[  1] =   760; lookup[  2] =  1503; lookup[  3] =  2228;
    lookup[  4] =  2936; lookup[  5] =  3627; lookup[  6] =  4303; lookup[  7] =  4963;
    lookup[  8] =  5609; lookup[  9] =  6240; lookup[ 10] =  6858; lookup[ 11] =  7462;
    lookup[ 12] =  8053; lookup[ 13] =  8631; lookup[ 14] =  9198; lookup[ 15] =  9752;
    lookup[ 16] = 10296; lookup[ 17] = 10828; lookup[ 18] = 11349; lookup[ 19] = 11860;
    lookup[ 20] = 12361; lookup[ 21] = 12852; lookup[ 22] = 13334; lookup[ 23] = 13807;
    lookup[ 24] = 14270; lookup[ 25] = 14725; lookup[ 26] = 15171; lookup[ 27] = 15609;
    lookup[ 28] = 16039; lookup[ 29] = 16461; lookup[ 30] = 16876; lookup[256] =     0;
    lookup[257] =   439; lookup[258] =   874; lookup[259] =  1306; lookup[260] =  1735;
    lookup[261] =  2160; lookup[262] =  2581; lookup[263] =  2999; lookup[264] =  3414;
    lookup[265] =  3826; lookup[266] =  4234; lookup[267] =  4639; lookup[268] =  5041;
    lookup[269] =  5440; lookup[270] =  5836; lookup[271] =  6229; lookup[272] =  6618;
    lookup[273] =  7005; lookup[274] =  7389; lookup[275] =  7769; lookup[276] =  8147;
    lookup[277] =  8522; lookup[278] =  8895; lookup[279] =  9264; lookup[280] =  9631;
    lookup[281] =  9995; lookup[282] = 10356; lookup[283] = 10714; lookup[284] = 11070;
    lookup[285] = 11423; lookup[286] = 11774; lookup[287] = 12122; lookup[288] = 12468;
    lookup[289] = 12811; lookup[290] = 13152; lookup[291] = 13490; lookup[292] = 13825;
    lookup[293] = 14159; lookup[294] = 14490; lookup[295] = 14818; lookup[296] = 15145;
    lookup[297] = 15469; lookup[298] = 15791; lookup[299] = 16110; lookup[300] = 16427;
    lookup[301] = 16742; lookup[302] = 17055; lookup[303] = 17366; lookup[304] = 17675;
    lookup[305] = 17981; lookup[306] = 18286; lookup[307] = 18588; lookup[308] = 18888;
    lookup[309] = 19187; lookup[310] = 19483; lookup[311] = 19777; lookup[312] = 20069;
    lookup[313] = 20360; lookup[314] = 20648; lookup[315] = 20935; lookup[316] = 21219;
    lookup[317] = 21502; lookup[318] = 21783; lookup[319] = 22062; lookup[320] = 22339;
    lookup[321] = 22615; lookup[322] = 22889; lookup[323] = 23160; lookup[324] = 23431;
    lookup[325] = 23699; lookup[326] = 23966; lookup[327] = 24231; lookup[328] = 24494;
    lookup[329] = 24756; lookup[330] = 25016; lookup[331] = 25274; lookup[332] = 25531;
    lookup[333] = 25786; lookup[334] = 26040; lookup[335] = 26292; lookup[336] = 26542;
    lookup[337] = 26791; lookup[338] = 27039; lookup[339] = 27284; lookup[340] = 27529;
    lookup[341] = 27772; lookup[342] = 28013; lookup[343] = 28253; lookup[344] = 28492;
    lookup[345] = 28729; lookup[346] = 28964; lookup[347] = 29198; lookup[348] = 29431;
    lookup[349] = 29663; lookup[350] = 29893; lookup[351] = 30121; lookup[352] = 30349;
    lookup[353] = 30575; lookup[354] = 30800; lookup[355] = 31023; lookup[356] = 31245;
    lookup[357] = 31466; lookup[358] = 31685; lookup[359] = 31904; lookup[360] = 32121;
    lookup[361] = 32336; lookup[362] = 32551; lookup[363] = 32764; lookup[364] = 32976;
    lookup[365] = 33187; lookup[366] = 33397; lookup[367] = 33605; lookup[368] = 33813;
    lookup[369] = 34019; lookup[370] = 34224; lookup[371] = 34428; lookup[372] = 34630;
    lookup[373] = 34832; lookup[374] = 35032; lookup[375] = 35232; lookup[376] = 35430;
    lookup[377] = 35627; lookup[378] = 35823; lookup[379] = 36018; lookup[380] = 36212;
    lookup[381] = 36405; lookup[382] = 36597; lookup[383] = 36788; lookup[384] = 36978;
    lookup[385] = 37166; lookup[386] = 37354; lookup[387] = 37541; lookup[388] = 37727;
    lookup[389] = 37912; lookup[390] = 38095; lookup[391] = 38278; lookup[392] = 38460;
    lookup[393] = 38641; lookup[394] = 38821; lookup[395] = 39000; lookup[396] = 39178;
    lookup[397] = 39355; lookup[398] = 39532; lookup[399] = 39707; lookup[400] = 39881;
    lookup[401] = 40055; lookup[402] = 40228; lookup[403] = 40399; lookup[404] = 40570;
    lookup[405] = 40740; lookup[406] = 40909; lookup[407] = 41078; lookup[408] = 41245;
    lookup[409] = 41412; lookup[410] = 41577; lookup[411] = 41742; lookup[412] = 41906;
    lookup[413] = 42070; lookup[414] = 42232; lookup[415] = 42394; lookup[416] = 42555;
    lookup[417] = 42715; lookup[418] = 42874; lookup[419] = 43032; lookup[420] = 43190;
    lookup[421] = 43347; lookup[422] = 43503; lookup[423] = 43659; lookup[424] = 43813;
    lookup[425] = 43967; lookup[426] = 44120; lookup[427] = 44273; lookup[428] = 44424;
    lookup[429] = 44575; lookup[430] = 44726; lookup[431] = 44875; lookup[432] = 45024;
    lookup[433] = 45172; lookup[434] = 45319; lookup[435] = 45466; lookup[436] = 45612;
    lookup[437] = 45757; lookup[438] = 45902; lookup[439] = 46046; lookup[440] = 46189;
    lookup[441] = 46332; lookup[442] = 46474; lookup[443] = 46615; lookup[444] = 46756;
    lookup[445] = 46895; lookup[446] = 47035; lookup[447] = 47173; lookup[448] = 47312;
    lookup[449] = 47449; lookup[450] = 47586; lookup[451] = 47722; lookup[452] = 47857;
    lookup[453] = 47992; lookup[454] = 48127; lookup[455] = 48260; lookup[456] = 48393;
    lookup[457] = 48526; lookup[458] = 48658;  
  end
  always @(posedge clk) begin
    tmp_a <= lookup[{1'b0, in_a}];
    tmp_b <= lookup[{1'b1, in_b}];
  end
  assign out = tmp_a + tmp_b;
endmodule


module APU(input clk, input ce, input reset,
           input [4:0] ADDR,  // APU Address Line
           input [7:0] DIN,   // Data to APU
           output [7:0] DOUT, // Data from APU
           input MW,          // Writes to APU
           input MR,          // Reads from APU
           input [4:0] audio_channels, // Enabled audio channels
           output [15:0] Sample,

           output DmaReq,      // 1 when DMC wants DMA
           input DmaAck,           // 1 when DMC byte is on DmcData. DmcDmaRequested should go low.
           output [15:0] DmaAddr,  // Address DMC wants to read
           input [7:0] DmaData,    // Input data to DMC from memory.

           output odd_or_even,
           output IRQ);       // IRQ asserted

// Which channels are enabled?
reg [3:0] Enabled;

// Output samples from the 4 channels
wire [3:0] Sq1Sample,Sq2Sample,TriSample,NoiSample;

// Output samples from the DMC channel
wire [6:0] DmcSample;
wire DmcIrq;
wire IsDmcActive;

// Generate internal memory write signals
wire ApuMW0 = MW && ADDR[4:2]==0; // SQ1
wire ApuMW1 = MW && ADDR[4:2]==1; // SQ2
wire ApuMW2 = MW && ADDR[4:2]==2; // TRI
wire ApuMW3 = MW && ADDR[4:2]==3; // NOI
wire ApuMW4 = MW && ADDR[4:2]>=4; // DMC
wire ApuMW5 = MW && ADDR[4:2]==5; // Control registers

wire Sq1NonZero, Sq2NonZero, TriNonZero, NoiNonZero;

// Common input to all channels
wire [7:0] LenCtr_In;
LenCtr_Lookup len(DIN[7:3], LenCtr_In);


// Frame sequencer registers
reg FrameSeqMode;
reg [15:0] Cycles;
reg ClkE, ClkL;
reg Wrote4017;
reg [1:0] IrqCtr;
reg InternalClock; // APU Differentiates between Even or Odd clocks
assign odd_or_even = InternalClock;


// Generate each channel
SquareChan   Sq1(clk, ce, reset, 0, ADDR[1:0], DIN, ApuMW0, ClkL, ClkE, Enabled[0], LenCtr_In, Sq1Sample, Sq1NonZero);
SquareChan   Sq2(clk, ce, reset, 1, ADDR[1:0], DIN, ApuMW1, ClkL, ClkE, Enabled[1], LenCtr_In, Sq2Sample, Sq2NonZero);
TriangleChan Tri(clk, ce, reset, ADDR[1:0], DIN, ApuMW2, ClkL, ClkE, Enabled[2], LenCtr_In, TriSample, TriNonZero);
NoiseChan    Noi(clk, ce, reset, ADDR[1:0], DIN, ApuMW3, ClkL, ClkE, Enabled[3], LenCtr_In, NoiSample, NoiNonZero);
DmcChan      Dmc(clk, ce, reset, odd_or_even, ADDR[2:0], DIN, ApuMW4, DmcSample, DmaReq, DmaAck, DmaAddr, DmaData, DmcIrq, IsDmcActive);

// Reading this register clears the frame interrupt flag (but not the DMC interrupt flag).
// If an interrupt flag was set at the same moment of the read, it will read back as 1 but it will not be cleared.
reg FrameInterrupt, DisableFrameInterrupt;


//mode 0: 4-step  effective rate (approx)
//---------------------------------------
//    - - - f      60 Hz
//    - l - l     120 Hz
//    e e e e     240 Hz


//mode 1: 5-step  effective rate (approx)
//---------------------------------------
//    - - - - -   (interrupt flag never set)
//    l - l - -    96 Hz
//    e e e e -   192 Hz

   
always @(posedge clk) if (reset) begin
  FrameSeqMode <= 0;
  DisableFrameInterrupt <= 0;
  FrameInterrupt <= 0;
  Enabled <= 0;
  InternalClock <= 0;
  Wrote4017 <= 0;
  ClkE <= 0;
  ClkL <= 0;
  Cycles <= 4; // This needs to be 5 for proper power up behavior
  IrqCtr <= 0;
end else if (ce) begin   
  FrameInterrupt <= IrqCtr[1] ? 1 : (ADDR == 5'h15 && MR || ApuMW5 && ADDR[1:0] == 3 && DIN[6]) ? 0 : FrameInterrupt;
  InternalClock <= !InternalClock;
  IrqCtr <= {IrqCtr[0], 1'b0};
  Cycles <= Cycles + 1;
  ClkE <= 0;
  ClkL <= 0;
  if (Cycles == 7457) begin
    ClkE <= 1;
  end else if (Cycles == 14913) begin
    ClkE <= 1;
    ClkL <= 1;
    ClkE <= 1;
    ClkL <= 1;
  end else if (Cycles == 22371) begin
    ClkE <= 1;
  end else if (Cycles == 29829) begin
    if (!FrameSeqMode) begin
      ClkE <= 1;
      ClkL <= 1;
      Cycles <= 0;
      IrqCtr <= 3;
      FrameInterrupt <= 1;
    end
  end else if (Cycles == 37281) begin
    ClkE <= 1;
    ClkL <= 1;
    Cycles <= 0;
  end
  
  // Handle one cycle delayed write to 4017.
  Wrote4017 <= 0;
  if (Wrote4017) begin
    if (FrameSeqMode) begin
      ClkE <= 1;
      ClkL <= 1;
    end
    Cycles <= 0;
  end
  
//  if (ClkE||ClkL) $write("%d: Clocking %s%s\n", Cycles, ClkE?"E":" ", ClkL?"L":" ");

  // Handle writes to control registers
  if (ApuMW5) begin
    case (ADDR[1:0])
    1: begin // Register $4015
      Enabled <= DIN[3:0];
//      $write("$4015 = %X\n", DIN);
    end
    3: begin // Register $4017
      FrameSeqMode <= DIN[7]; // 1 = 5 frames cycle, 0 = 4 frames cycle
      DisableFrameInterrupt <= DIN[6];
     
      // If the internal clock is even, things happen
      // right away.
      if (!InternalClock) begin
        if (DIN[7]) begin
          ClkE <= 1;
          ClkL <= 1;
        end
        Cycles <= 0;
      end
      
      // Otherwise they get delayed one clock
      Wrote4017 <= InternalClock;
    end
    endcase
  end


end

ApuLookupTable lookup(clk, 
                      (audio_channels[0] ? {4'b0, Sq1Sample} : 8'b0) + 
                      (audio_channels[1] ? {4'b0, Sq2Sample} : 8'b0), 
                      (audio_channels[2] ? {4'b0, TriSample} + {3'b0, TriSample, 1'b0} : 8'b0) + 
                      (audio_channels[3] ? {3'b0, NoiSample, 1'b0} : 8'b0) +
                      (audio_channels[4] ? {1'b0, DmcSample} : 8'b0),
                      Sample);

wire frame_irq = FrameInterrupt && !DisableFrameInterrupt;

// Generate bus output
assign DOUT = {DmcIrq, frame_irq, 1'b0, 
               IsDmcActive,
               NoiNonZero,
               TriNonZero,
               Sq2NonZero,
               Sq1NonZero};

assign IRQ = frame_irq || DmcIrq;

endmodule