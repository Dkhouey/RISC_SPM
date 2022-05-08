`timescale 1ns/1ps
module tb_RISC_SPM #(parameter word_size = 8)();
 
reg rst;
wire clk, data_bus, address_bus, instruction_bus, memory_bus;
reg [8:0] k;
 
Clock_Unit M1 (clk);
RISC_SPM M2 (clk, rst, data_bus, address_bus, instruction_bus, memory_bus);
 
// define probes
wire [word_size-1:0] word0, word1, word2, word3, word4, word5, word6,
                    word7, word8, word9, word10, word11, word12, word13,
                    word14, word128, word129, word130, word131, word132,
                    word133, word134, word135, word136, word137, word255,
                    word138, word139, word140;
 
assign
       word0 = M2.M2_MEM.memory[0],
       word1 = M2.M2_MEM.memory[1],
       word2 = M2.M2_MEM.memory[2],
       word3 = M2.M2_MEM.memory[3],
       word4 = M2.M2_MEM.memory[4],
       word5 = M2.M2_MEM.memory[5],
       word6 = M2.M2_MEM.memory[6],
       word7 = M2.M2_MEM.memory[7],
       word8 = M2.M2_MEM.memory[8],
       word9 = M2.M2_MEM.memory[9],
       word10 = M2.M2_MEM.memory[10],
       word11 = M2.M2_MEM.memory[11],
       word12 = M2.M2_MEM.memory[12],
       word13 = M2.M2_MEM.memory[13],
       word14 = M2.M2_MEM.memory[14],
       word128 = M2.M2_MEM.memory[128],
       word129 = M2.M2_MEM.memory[129],
       word130 = M2.M2_MEM.memory[130],
       word131 = M2.M2_MEM.memory[131],
       word132 = M2.M2_MEM.memory[132],
       word133 = M2.M2_MEM.memory[133],
       word134 = M2.M2_MEM.memory[134],
       word135 = M2.M2_MEM.memory[135],
       word136 = M2.M2_MEM.memory[136],
       word137 = M2.M2_MEM.memory[137],
       word138 = M2.M2_MEM.memory[138],
       word140 = M2.M2_MEM.memory[140],
       word255 = M2.M2_MEM.memory[255];
 
initial
begin
       $dumpfile("tb_RISC_SPM.vcd");
       $dumpvars(0, tb_RISC_SPM);
#60000 $finish;
 
end
 
 
// Flush Memory
initial begin: Flush_Memory
#2 rst = 0; for ( k = 0; k <= 255; k = k+1) M2.M2_MEM.memory[k] = 0; #10 rst = 1;
end
 
initial begin: Load_program
#5
                                        // opcode_src_dest
M2.M2_MEM.memory[0] = 8'b0101_00_01;  // Read 130 to R1
M2.M2_MEM.memory[1] = 130;
M2.M2_MEM.memory[2] = 8'b0001_00_01;  // Add R0+R1
M2.M2_MEM.memory[3] = 8'b0000_00_00;  // NOP
M2.M2_MEM.memory[4] = 8'b0101_00_10;  // Read 128 to R2
M2.M2_MEM.memory[5] = 128;
M2.M2_MEM.memory[6] = 8'b0010_01_10;  // Sub R2-R1
M2.M2_MEM.memory[7] = 8'b1000_00_00;  // BRZ
M2.M2_MEM.memory[8] = 129;            // Holds address for BRZ
M2.M2_MEM.memory[9] = 8'b0001_10_11;  // Add R2+R3 to R3
M2.M2_MEM.memory[10] = 8'b0000_00_00; // NOP
//M2.M2_MEM.memory[11] = 134;           // Holds address for BRZ
//M2.M2_MEM.memory[12] = 8'b0001_10_11; // Add R2+R3 to R3
//M2.M2_MEM.memory[13] = 8'b0111_00_11; // BR
//M2.M2_MEM.memory[14] = 140;
 
                                      // opcode_src_dest
//M2.M2_MEM.memory[0] = 8'b0000_00_00;  // NOP
//M2.M2_MEM.memory[1] = 8'b0101_00_10;  // Read 130 to R2
//M2.M2_MEM.memory[2] = 130;
//M2.M2_MEM.memory[3] = 8'b0101_00_11;  // Read 131 to R3
//M2.M2_MEM.memory[4] = 131;
//M2.M2_MEM.memory[5] = 8'b0101_00_01;  // Read 128 to R1
//M2.M2_MEM.memory[6] = 128;
//M2.M2_MEM.memory[7] = 8'b0101_00_00;  // Read 129 to R0
//M2.M2_MEM.memory[8] = 129;
//M2.M2_MEM.memory[9] = 8'b0010_00_01;  // Sub R1-R0 to R1
//M2.M2_MEM.memory[10] = 8'b1000_00_00; // BRZ
//M2.M2_MEM.memory[11] = 134;           // Holds address for BRZ
//M2.M2_MEM.memory[12] = 8'b0001_10_11; // Add R2+R3 to R3
//M2.M2_MEM.memory[13] = 8'b0111_00_11; // BR
//M2.M2_MEM.memory[14] = 140;
// Load data
M2.M2_MEM.memory[128] = 6;
M2.M2_MEM.memory[129] =1;
M2.M2_MEM.memory[130] = 2;
M2.M2_MEM.memory[131] = 0;
M2.M2_MEM.memory[134] = 139;
//M2.M2_MEM.memory[139] = 8'b1111_00_00; // HALT
M2.M2_MEM.memory[140] = 9;             // Recycle
 
end
 
endmodule

module Clock_Unit (output reg clock);
 
parameter delay = 0;
parameter half_cycle = 10;
initial begin #delay clock = 0; forever #half_cycle clock = ~clock;
 
end
 
endmodule

   
