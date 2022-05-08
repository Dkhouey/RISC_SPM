module dump();
    initial begin
        $dumpfile ("wb_RISC_SPM.vcd");
        $dumpvars (0, wb_RISC_SPM);
        #1;
    end
endmodule
