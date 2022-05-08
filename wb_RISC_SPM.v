`default_nettype none
`timescale 1ns/1ns

module wb_RISC_SPM #(
    parameter   [31:0]  BASE_ADDRESS     = 32'h3000_0000,        // base address
    parameter   [31:0]  INPUT_ADDRESS     = BASE_ADDRESS,
    parameter   [31:0]  OUTPUT_ADDRESS    = BASE_ADDRESS + 4
    )(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
    input wire          clk,
    input wire          reset,
    // wb interface
    input wire          i_wb_cyc,       // wishbone transaction
    input wire          i_wb_stb,       // strobe - data valid and accepted as long as !o_wb_stall
    input wire          i_wb_we,        // write enable
    input wire  [31:0]  i_wb_addr,      // address
    input wire  [31:0]  i_wb_data,      // incoming data
    output reg          o_wb_ack,       // request is completed
    output wire         o_wb_stall,     // cannot accept req
    output reg  [31:0]  o_wb_data      // output data
);

    reg [7:0] address_bus;
    reg [7:0] data_bus;
    reg [7:0] instruction_bus;
    reg [7:0] memory_bus;
    RISC_SPM RISC_SPM_0(
                                      .address_bus(address_bus),
                                      .data_bus(data_bus),
                                      .instruction_bus(instruction_bus),
                                      .memory_bus(memory_bus),
                                      .clk(clk),
                                      .reset(reset));



    // taking input
    always @(posedge clk) begin
        if(reset) begin
            instruction_bus <= 4'b0;
            memory_bus <= 4'b0;
        end
        else if(i_wb_stb && i_wb_cyc && i_wb_we && !o_wb_stall && i_wb_addr == INPUT_ADDRESS) begin
            instruction_bus <= i_wb_data[3:0];
            memory_bus <= i_wb_data[7:4];
        end
    end
        // putting output
        always @(posedge clk) begin
            if(reset)
                o_wb_data <= 0;
            else if(i_wb_stb && i_wb_cyc && !i_wb_we && !o_wb_stall)
                case(i_wb_addr)
                    INPUT_ADDRESS:
                        o_wb_data <= {memory_bus,instruction_bus};
                    OUTPUT_ADDRESS:
                        o_wb_data <= {data_bus,address_bus};
                    default:
                        o_wb_data <= 32'b0;
                endcase
        end

    // acks
    always @(posedge clk) begin
        if(reset)
            o_wb_ack <= 0;
        else
            begin
                o_wb_ack <= (i_wb_stb && !o_wb_stall && (i_wb_addr == OUTPUT_ADDRESS || i_wb_addr == INPUT_ADDRESS));
            end
    end
endmodule 
    

module RISC_SPM (clk, rst, data_bus, address_bus, memory_bus, instruction_bus);

   parameter word_size = 8;
   parameter Sel1_size = 3; 
   parameter Sel2_size = 2;
   input clk, rst;
   output [word_size - 1:0] data_bus, address_bus, memory_bus, instruction_bus;



// Data Nets
wire [Sel1_size -1:0] Sel_Bus_1_Mux;
wire [Sel2_size -1:0] Sel_Bus_2_Mux;
wire zero;
wire [word_size -1:0] instruction, address, Bus_1, mem_word;
assign data_bus= Bus_1;
assign address_bus= address;
assign memory_bus= mem_word;
assign instruction_bus= instruction;
 
// Control Nets
wire Load_R0, Load_R1, Load_R2, Load_R3, Load_PC, Inc_PC, Load_IR,
    Load_Add_R, Load_Reg_Y, Load_Reg_Z, write;
 
Processing_Unit M0_Processor (instruction, address, Bus_1, zero, mem_word,
Load_R0, Load_R1, Load_R2, Load_R3, Load_PC, Inc_PC, Sel_Bus_1_Mux,
Sel_Bus_2_Mux, Load_IR, Load_Add_R, Load_Reg_Y, Load_Reg_Z, clk, rst);
 
Control_Unit M1_Controller (Sel_Bus_2_Mux, Sel_Bus_1_Mux, Load_R0,
Load_R1, Load_R2, Load_R3, Load_PC, Inc_PC, Load_IR, Load_Add_R,
Load_Reg_Y, Load_Reg_Z, write, instruction, zero, clk, rst);
 
Memory_Unit M2_MEM (
   .data_out(mem_word),
   .data_in(Bus_1),
   .address(address),
   .clk(clk),
   .write(write)
);

endmodule

module Processing_Unit #(parameter
word_size = 8, op_size = 4, Sel1_size = 3, Sel2_size = 2)(
output [word_size -1:0] instruction, address, Bus_1,
output                  Zflag,
input [word_size -1:0]  mem_word,
input                   Load_R0, Load_R1, Load_R2, Load_R3, Load_PC, Inc_PC,
input [Sel1_size -1:0]  Sel_Bus_1_Mux,
input [Sel2_size -1:0]  Sel_Bus_2_Mux,
input                   Load_IR, Load_Add_R, Load_Reg_Y, Load_Reg_Z,
input                   clk, rst
);
 
wire [word_size -1:0]   Bus_2;
wire [word_size -1:0]   R0_out, R1_out, R2_out, R3_out;
 
wire [word_size -1:0]  PC_count, Y_value, alu_out;
wire                   alu_zero_flag;
wire [op_size -1:0]    opcode = instruction [word_size-1: word_size-op_size];
 
Register_Unit         R0     (R0_out, Bus_2, Load_R0, clk, rst);
Register_Unit         R1     (R1_out, Bus_2, Load_R1, clk, rst);
Register_Unit         R2     (R2_out, Bus_2, Load_R2, clk, rst);
Register_Unit         R3     (R3_out, Bus_2, Load_R3, clk, rst);
Register_Unit         Reg_Y  (Y_value, Bus_2, Load_Reg_Y, clk, rst);
D_flop                Reg_Z  (Zflag, alu_zero_flag, Load_Reg_Z, clk, rst);
 
Address_Register      Add_R  (address, Bus_2, Load_Add_R, clk, rst);
Instruction_Register  IR     (instruction, Bus_2, Load_IR, clk, rst);
Program_Counter       PC     (PC_count, Bus_2, Load_PC, Inc_PC, clk, rst);
 
Multiplexer_5ch       Mux_1  (Bus_1, R0_out, R1_out, R2_out, R3_out, PC_count, Sel_Bus_1_Mux);
Multiplexer_3ch       Mux_2  (Bus_2, alu_out, Bus_1, mem_word, Sel_Bus_2_Mux);
 
Alu_RISC              ALU    (alu_out, alu_zero_flag, Y_value, Bus_1, opcode);
 
endmodule
 
 
 
 
 
module Register_Unit #(parameter word_size = 8)(
output reg [word_size-1:0]  data_out,
input [word_size -1:0]      data_in,
input                       load, clk, rst
);
 
always @(posedge clk, negedge rst)
 
begin
 
if (rst == 1'b0) data_out <= 0; else if (load) data_out <= data_in;
 
end
 
endmodule
 
 
module D_flop (output reg data_out, input data_in, load, clk, rst);
 
always @(posedge clk, negedge rst)
 
begin
 
if (rst == 1'b0) data_out <= 0; else if (load == 1'b1) data_out <= data_in;
 
end
 
endmodule
 
 
module Address_Register #(parameter word_size = 8)(
output reg [word_size -1:0] data_out,
input [word_size -1:0]      data_in,
input                       load, clk, rst
);
 
always @(posedge clk, negedge rst)
 
begin
 
if (rst == 1'b0) data_out <= 0; else if (load == 1'b1) data_out <= data_in;
 
end
 
endmodule
 
 
module Instruction_Register #(parameter word_size = 8)(
output reg [word_size -1:0] data_out,
input [word_size -1:0]      data_in,
input                       load, clk, rst
);
 
always @(posedge clk, negedge rst)
 
begin
 
   if (rst == 1'b0) data_out <= 0; else if (load == 1'b1) data_out <= data_in;
 
end
 
endmodule
 
 
module Program_Counter #(parameter word_size = 8)(
output reg [word_size -1:0]  count,
input [word_size -1:0]       data_in,
input                        Load_PC, Inc_PC,
input                        clk, rst
);
 
always @(posedge clk, negedge rst)
 
begin
 
   if (rst == 1'b0) count <= 0;
   else if (Load_PC == 1'b1) count <= data_in;
   else if (Inc_PC == 1'b1) count <= count +1;
  
end
  
endmodule
 
 
module Multiplexer_5ch #(parameter word_size = 8)(
output [word_size -1:0]  mux_out,
input [word_size -1:0]   data_a, data_b, data_c, data_d, data_e,
input [2:0]              sel
);
 
assign mux_out = (sel == 0)   ? data_a: (sel == 1)
                             ? data_b: (sel == 2)
                             ? data_c: (sel == 3)
                             ? data_d: (sel == 4)
                             ? data_e: 'bx;
                          
endmodule
 
 
module Multiplexer_3ch #(parameter word_size = 8)(
output [word_size -1:0]  mux_out,
input [word_size -1:0]   data_a, data_b, data_c,
input [1:0]              sel
);
  
assign mux_out = (sel == 0) ? data_a: (sel == 1) ? data_b: (sel ==2) ? data_c: 'bx;
 
endmodule
 
 
module Alu_RISC #(parameter word_size = 8,op_size = 4,
//Opcodes
NOP  =4'b0000,
ADD  =4'b0001,
SUB  =4'b0010,
AND  =4'b0011,
NOT  =4'b0100,
RD   =4'b0101,
WR   =4'b0110,
BR   =4'b0111,
BRZ  =4'b1000
)
 
(
output reg [word_size -1:0]  alu_out,
output                       alu_zero_flag,
input [word_size -1:0]       data_1, data_2,
input [op_size -1:0]         sel
);
 
assign alu_zero_flag = ~|alu_out;
 
always @(sel, data_1, data_2)
 
case (sel)
 NOP:      alu_out = 0;
 ADD:      alu_out = data_1 + data_2; //Reg_Y + Bus_1
 SUB:      alu_out = data_2 - data_1;
 AND:      alu_out = data_1 & data_2;
 NOT:      alu_out = ~ data_2;       // Gets data from Bus_1
 default:  alu_out= 0;
 
endcase
 
endmodule
 
 
module Control_Unit #(parameter
word_size = 8, op_size = 4, state_size = 4,
src_size = 2, dest_size = 2, Sel1_size = 3, Sel2_size = 2)(
output [Sel2_size -1:0] Sel_Bus_2_Mux,
output [Sel1_size -1:0] Sel_Bus_1_Mux,
output reg Load_R0, Load_R1, Load_R2, Load_R3, Load_PC, Inc_PC,
           Load_IR, Load_Add_R, Load_Reg_Y, Load_Reg_Z, write,
input [word_size -1:0] instruction,
input zero, clk, rst
);
 
// State Codes
parameter S_idle = 0, S_fet1 = 1, S_fet2 = 2, S_dec = 3,
          S_ex1 = 4, S_rd1 = 5, S_rd2 = 6,
          S_wr1 = 7, S_wr2 = 8, S_br1 = 9, S_br2 = 10, S_halt = 11;
 
// Opcodes
parameter NOP = 0, ADD = 1, SUB = 2, AND = 3, NOT = 4, RD = 5, WR = 6,
          BR = 7, BRZ = 8;
 
// Source and Destination Codes
parameter R0 = 0, R1 = 1, R2 = 2, R3 = 3;
 
reg [state_size -1:0] state, next_state;
reg Sel_ALU, Sel_Bus_1, Sel_Mem;
reg Sel_R0, Sel_R1, Sel_R2, Sel_R3, Sel_PC;
reg err_flag;
wire [op_size -1:0] opcode = instruction [word_size -1: word_size - op_size];
wire [src_size -1:0] src = instruction [src_size + dest_size -1: dest_size];
wire [dest_size -1:0] dest = instruction [dest_size -1:0];
 
// Mux selectors
assign Sel_Bus_1_Mux [Sel1_size -1:0] = Sel_R0 ? 0:
                                        Sel_R1 ? 1:
                                        Sel_R2 ? 2:
                                        Sel_R3 ? 3:
                                        Sel_PC ? 4: 3'bx; //3-bits, sized number
 
assign Sel_Bus_2_Mux [Sel2_size -1:0] = Sel_ALU ? 0:
                                        Sel_Bus_1 ? 1:
                                        Sel_Mem ? 2: 2'bx;
 
always @(posedge clk, negedge rst)
begin: State_transitions
 
 if (rst == 0) state <= S_idle;
 else state <= next_state;
   
end
 
always @(state, opcode, src, dest, zero)
 
begin: Output_and_next_state
 
 Sel_R0 = 0; Sel_R1 = 0; Sel_R2 = 0; Sel_R3 = 0; Sel_PC = 0;
 Load_R0 = 0; Load_R1 = 0; Load_R2 = 0; Load_R3 =0; Load_PC = 0;
 Load_IR = 0; Load_Add_R = 0; Load_Reg_Y = 0; Load_Reg_Z = 0;
 Inc_PC = 0;
 Sel_Bus_1 = 0;
 Sel_ALU = 0;
 Sel_Mem = 0;
 write = 0;
 err_flag = 0; // Used for de-bug in simulation
 
next_state = state;
 
case (state) S_idle: next_state = S_fet1;
             S_fet1:
                    begin
                        next_state = S_fet2;
                        Sel_PC = 1;
                        Sel_Bus_1 = 1;
                        Load_Add_R = 1;
                    end
            
             S_fet2:
                    begin
                        next_state = S_dec;
                        Sel_Mem = 1;
                        Load_IR = 1;
                        Inc_PC = 1;
                    end
 
             S_dec:
case (opcode)
 NOP: next_state = S_fet1;
 ADD, SUB, AND:
               begin
                   next_state = S_ex1;
                   Sel_Bus_1 = 1;
                   Load_Reg_Y = 1;
case (src)
 
 R0:       Sel_R0 = 1;
 R1:       Sel_R1 = 1;
 R2:       Sel_R2 = 1;
 R3:       Sel_R3 = 1;
 default   err_flag = 1;
 
 endcase
 
               end // ADD, SUB, AND
 
NOT:
    begin
        next_state = S_fet1;
        Load_Reg_Z = 1;
        Sel_ALU = 1;
 
case (src)
  R0:       Sel_R0 = 1;
 R1:       Sel_R1 = 1;
 R2:       Sel_R2 = 1;
 R3:       Sel_R3 = 1;
 default   err_flag = 1;
 
endcase
 
case (dest)
 
 R0:       Load_R0 = 1;
 R1:       Load_R1 = 1;
 R2:       Load_R2 = 1;
 R3:       Load_R3 = 1;
 default   err_flag = 1;
 
endcase
 
    end // NOT
 
RD:
   begin
       next_state = S_rd1;
       Sel_PC = 1; Sel_Bus_1 = 1; Load_Add_R = 1;
  
   end // RD
 
WR:
   begin
       next_state = S_wr1;
       Sel_PC = 1; Sel_Bus_1 = 1; Load_Add_R = 1;
  
   end // WR
 
BR:
   begin
       next_state = S_br1;
       Sel_PC = 1; Sel_Bus_1 = 1; Load_Add_R = 1;
  
   end // BR
 
BRZ: if (zero == 1)
    begin
        next_state = S_br1;
        Sel_PC = 1; Sel_Bus_1 = 1; Load_Add_R = 1;
  
    end // BRZ
 
   else
       begin
           next_state = S_fet1;
           Inc_PC = 1;
 
       end
  
   default: next_state = S_halt;
 
   endcase // (opcode)
 
S_ex1:
      begin
          next_state = S_fet1;
          Load_Reg_Z = 1;
          Sel_ALU = 1;
 
       case (dest)
           R0: begin Sel_R0 = 1; Load_R0 = 1; end
           R1: begin Sel_R1 = 1; Load_R1 = 1; end
           R2: begin Sel_R2 = 1; Load_R2 = 1; end
           R3: begin Sel_R3 = 1; Load_R3 = 1; end
 
           default: err_flag = 1;
 
       endcase
 
      end
 
S_rd1:
      begin
          next_state = S_rd2;
          Sel_Mem = 1;
          Load_Add_R = 1;
          Inc_PC = 1;
 
      end
 
S_wr1:
      begin
          next_state = S_wr2;
          Sel_Mem = 1;
          Load_Add_R = 1;
          Inc_PC = 1;
 
      end
 
S_rd2:
      begin
          next_state = S_fet1;
          Sel_Mem = 1;
      
       case (dest)
         R0:      Load_R0 = 1;
         R1:      Load_R1 = 1;
         R2:      Load_R2 = 1;
         R3:      Load_R3 = 1;
 
         default  err_flag = 1;
 
       endcase
 
      end
 
S_wr2:
      begin
          next_state = S_fet1;
          write = 1;
 
       case (src)
         R0:      Sel_R0 = 1;
         R1:      Sel_R1 = 1;
         R2:      Sel_R2 = 1;
         R3:      Sel_R3 = 1;
 
         default  err_flag = 1;
 
       endcase
 
      end
 
S_br1:
      begin
          next_state = S_br2; Sel_Mem = 1;
          Load_Add_R = 1;
 
      end
 
S_br2:
      begin
          next_state = S_fet1; Sel_Mem = 1;
          Load_PC = 1;
 
      end
 
S_halt: next_state = S_halt;
 
default: next_state = S_idle;
 
endcase
 
end
 
endmodule
 
 
module Memory_Unit #(parameter word_size = 8, memory_size = 256)(
  
output [word_size -1:0] data_out,
input [word_size -1:0] data_in,
input [word_size -1:0] address,
input clk, write  
);
 
reg [word_size -1:0] memory [memory_size -1:0];
 
assign data_out = memory [address];
 
always @(posedge clk)
begin
   
    if (write) memory [address] <= data_in;
end
 
endmodule

	
