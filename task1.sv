module task1(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);
  // your implementation here

    wire [15:0] sximm5, sximm8, mdata, ram_r_data, ram_w_data;
    wire [15:0] datapath_out;
    wire [15:0] ir;
    wire [1:0] reg_sel, ALU_op, shift_op, wb_sel;
    wire [2:0] opcode, r_addr, w_addr;
    wire [7:0] ram_r_addr, ram_w_addr;
    wire w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, load_ir, waiting, sel_addr, load_pc, clear_pc, load_addr;

    wire [7:0] next_pc, pc, dar_out;

    ram ram(.clk, .ram_w_en, .ram_r_addr, .ram_w_addr, .ram_w_data, .ram_r_data);
    ireg instr_reg(.clk, .instr(ram_r_data), .load_ir, .ir);
    idecoder instr_dec(.ir, .reg_sel, .opcode, .ALU_op, .shift_op, .sximm5, .sximm8, .r_addr, .w_addr);
    datapath dp(.clk, .mdata, .pc, .wb_sel, .w_addr, .w_en, .r_addr, .en_A, .en_B, .shift_op, .sel_A, .sel_B, .ALU_op, .en_C, .en_status, .sximm8, .sximm5, .datapath_out);
    controller control(.clk, .rst_n, .opcode, .ALU_op, .shift_op, .waiting, .reg_sel, .wb_sel, .w_en, .en_A, .en_B, .en_C, .en_status, .sel_A, .sel_B, .ram_w_en, .sel_addr, .load_pc, .clear_pc, .load_addr, .load_ir);

    assign ram_w_data = datapath_out;
    assign out = datapath_out;
    assign mdata = ram_r_data;

    assign next_pc = clear_pc? start_pc : pc + 1;
    reg_en8 PC(.clk, .in(next_pc), .en(load_pc), .out(pc));

    DAR dar(.clk, .in(datapath_out), .en(load_addr), .out(dar_out));

    assign ram_r_addr = sel_addr ? pc : dar_out;
    assign ram_w_addr = sel_addr ? pc : dar_out;
    

/*
module status(input clk, input en_status, output reg Z_out, output reg N_out, output reg V_out);

    always_ff @(posedge clk) begin
        if (en_status) begin
            Z_out <= Z;
            N_out <= N;
            V_out <= V;
        end
    end
endmodule: status*/


endmodule: task1

module ram(input clk, input ram_w_en, input [7:0] ram_r_addr, input [7:0] ram_w_addr,
           input [15:0] ram_w_data, output reg [15:0] ram_r_data);
    reg [15:0] m[255:0];
    always_ff @(posedge clk) begin
        if (ram_w_en) m[ram_w_addr] <= ram_w_data;
        ram_r_data <= m[ram_r_addr];
    end
    initial $readmemb("ram_init.txt", m);
endmodule: ram


    module reg_en8(input clk, input [7:0] in, input en, output reg [7:0] out); 
    always_ff @(posedge clk) begin
        if (en) begin
            out <= in;
        end
    end
    endmodule//: reg_en8

    module DAR(input clk, input [15:0] in, input en, output reg [7:0] out); 
    always_ff @(posedge clk) begin
        if (en) begin
            out <= in[7:0];
        end
    end
    endmodule//: DAR


module ireg(input clk, input [15:0] instr, input load_ir, output reg [15:0] ir);
    always_ff @(posedge clk) begin
        if (load_ir) begin
            ir <= instr;
        end
    end
endmodule//: ireg

module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output reg [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr);
  // your implementation here
  assign r_addr = (reg_sel == 2'b00) ? ir[2:0] : ((reg_sel == 2'b01) ? ir[7:5] : ((reg_sel == 2'b10)? ir[10:8] : 3'd0));
  assign w_addr = (reg_sel == 2'b00) ? ir[2:0] : ((reg_sel == 2'b01) ? ir[7:5] : ((reg_sel == 2'b10)? ir[10:8] : 3'd0)); 
  //assign shift_op = ir[4:3];
  assign sximm8 = $signed(ir[7:0]);
  assign sximm5 = $signed(ir[4:0]);
  assign ALU_op = ir[12:11];
  assign opcode = ir[15:13];

  always_comb begin 
    if(opcode == 3'b011 || opcode == 3'b100) shift_op = 2'b00;
    else shift_op = ir[4:3];
  end





endmodule//: idecoder



module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, input en_status,
		            input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out);
  
    reg [15:0] w_data;
    reg [15:0] r_data;
    reg [15:0] val_A;
    reg [15:0] val_B;
    reg [15:0] B_out;
    reg [15:0] input_A;
    reg [15:0] input_B;
    reg [15:0] ALU_out;
    //reg [15:0] mux_out;

    //writeback_mux mux_writeback(.mdata, .sximm8, .pc, .datapath_out, .mux_out(w_data), .wb_sel); 
    always_comb begin
        case(wb_sel)
        2'b00: w_data = datapath_out;
        2'b01: w_data = {8'd0, pc};
        2'b10: w_data = sximm8;
        2'b11: w_data = mdata;
        default: w_data = 16'bxxxxxxxxxxxxxxxx;
        endcase 
    end

    regfile register(.w_data, .w_addr, .w_en, .r_addr, .clk, .r_data);
    reg_en A(.clk, .in(r_data), .en(en_A), .out(input_A));
    reg_en B(.clk, .in(r_data), .en(en_B), .out(B_out));
    shifter shift(.shift_in(B_out), .shift_op, .shift_out(input_B));
    mux_A multiplexer_A(.sel_A, .A_in(input_A), .val_A);
    mux_B multiplexer_B(.sel_B, .B_in(input_B), .sximm5, .val_B);
    ALU alu(.val_A, .val_B, .ALU_op, .ALU_out);
    //status status_reg(.clk, .Z, .N, .V, .en_status, .Z_out, .N_out, .V_out);
    reg_en C(.clk, .in(ALU_out), .en(en_C), .out(datapath_out));

endmodule//: datapath

module writeback_mux(input [15:0] mdata, input [15:0] sximm8, input [7:0] pc, input [15:0] datapath_out, output reg [15:0] mux_out, input [1:0] wb_sel);  //datapath_out = C in figure
//can also use assign statment-- check later
always_comb begin
    case(wb_sel)
    2'b00: mux_out = datapath_out;
    2'b01: mux_out = {8'd0, pc};
    2'b10: mux_out = sximm8;
    2'b11: mux_out = mdata;
    default: mux_out = 16'bxxxxxxxxxxxxxxxx;
    endcase 
end
endmodule//: writeback_mux 

module regfile(input [15:0] w_data, input [2:0] w_addr, input w_en, input [2:0] r_addr, input clk, output [15:0] r_data);
    reg [15:0] m[0:7];
    assign r_data = m[r_addr];
    always_ff @(posedge clk) if (w_en) m[w_addr] <= w_data;
endmodule//: regfile

module reg_en(input clk, input [15:0] in, input en, output reg [15:0] out); 
    always_ff @(posedge clk) begin
        if (en) begin
            out <= in;
        end
    end
endmodule//: reg_en

module shifter(input [15:0] shift_in, input [1:0] shift_op, output reg [15:0] shift_out);
    always_comb begin
        case (shift_op) 
            2'b00: shift_out = shift_in;
            2'b01: shift_out = shift_in << 1;
            2'b10: shift_out = shift_in >> 1;
            2'b11: begin
                shift_out = shift_in >> 1;
                shift_out[15] = shift_in[15];
            end
            default: shift_out = 16'bxxxxxxxxxxxxxxxx;
        endcase
    end
endmodule//: shifter

module mux_A(input sel_A, input [15:0] A_in, output [15:0] val_A);   //A_in = A_out
    assign val_A = sel_A ? 16'd0 : A_in;
endmodule

module mux_B (input sel_B, input [15:0] B_in, input [15:0] sximm5, output [15:0] val_B); // B_in = shift_out
    assign val_B = sel_B ? sximm5 : B_in;
endmodule

module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out); 

 reg [15:0] alu_out;
 assign ALU_out = alu_out;

  always_comb begin 
        case(ALU_op)
        2'b00: alu_out = val_A + val_B;
        2'b01: alu_out = val_A - val_B;
        2'b10: alu_out = val_A & val_B;
        2'b11: alu_out = ~val_B;
        default: alu_out = 16'bxxxxxxxxxxxxxxxx;
        endcase
  end
endmodule//: ALU

module controller(input clk, input rst_n,
                  input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  output reg waiting,
                  output reg [1:0] reg_sel, output reg [1:0] wb_sel, output reg w_en,
                  output reg en_A, output reg en_B, output reg en_C, output reg en_status,
                  output reg sel_A, output reg sel_B,
                  output reg ram_w_en, output reg sel_addr, output reg load_pc,
                  output reg clear_pc, output reg load_addr, output reg load_ir);

  enum reg [4:0] {
    Reset = 5'b00000,
    Decode = 5'b00001,
    WriteImm8 = 5'b00010,
    StoreRn = 5'b00011,
    StoreRm = 5'b00100,
    Add = 5'b00101,
    Cmp = 5'b00110,
    And = 5'b00111,
    Mvn = 5'b01000,
    sh_Rm = 5'b01001,
    Write_final = 5'b01010,
    Fetch = 5'b01011,
    Load_IR = 5'b01100,
    Halt = 5'b01101,
    LDR_1 = 5'b01110,
    LDR_3 = 5'b01111,
    LDR_5 = 5'b10000,
    LDR_2 = 5'b10001,
    LDR_4 = 5'b10010,
    STR_1 = 5'b10011,
    STR_2 = 5'b10100,
    STR_3 = 5'b10101,
    sh_Rm1 = 5'b10110,
    sh_Rm2 = 5'b10111,
    Fetch2 = 5'b11000,
    STRx = 5'b11001,
    STRy = 5'b11010,
    Write_sh = 5'b11011} state;

    always_ff @(posedge clk) begin
      if (~rst_n) begin
          state <= Reset;
      end else begin
        
        case(state)
        Reset: state <= Fetch;

        Fetch: state <= Load_IR;       

        Load_IR: state <= Fetch2;

        Fetch2: if({opcode,ALU_op} == {3'b111, 2'b00}) state <= Halt;
        else state <= Decode;

        Decode: if(opcode == 3'b110 && ALU_op == 2'b10) state <= WriteImm8;
                else if(opcode != 3'b110 && opcode != 3'b101 && opcode != 3'b011 && opcode != 3'b100 && opcode != 3'b111) state <= Fetch;
                else if({opcode,ALU_op} == {3'b101, 2'b11} || {opcode,ALU_op} == {3'b110, 2'b00}) state <= sh_Rm1;
                else if({opcode,ALU_op} == {3'b111, 2'b00}) state <= Halt;
                else if (opcode == 3'b011 || opcode == 3'b100) state <= LDR_1;
                else if ({opcode,ALU_op} == {3'b101, 2'b00} || {opcode,ALU_op} == {3'b101, 2'b01} || {opcode,ALU_op} == {3'b101, 2'b10}) state <= StoreRn;
                else state <= Reset;

        WriteImm8: state <= Reset;

        StoreRn: state <= StoreRm;

        StoreRm: if(opcode == 3'b101 && ALU_op == 2'b00) state <= Add;
        else if (opcode == 3'b101 && ALU_op == 2'b01) state <= Cmp;
        else if (opcode == 3'b101 && ALU_op == 2'b10) state <= And;
        //else if (opcode == 3'b101 && ALU_op == 2'b11) state <= Mvn;
        else state <= sh_Rm1;

        sh_Rm1: state <= sh_Rm2;

        sh_Rm2: state <= Write_sh;

        Write_sh: state <= Reset;

        Add: state <= Write_final;

        Cmp: state <= Write_final;

        And: state <= Write_final;

        Write_final: state <= Reset;

        LDR_1: if(opcode == 3'b100) state <= STRx;
        else state <= LDR_2;

        LDR_2: state <= LDR_3;

        LDR_3: state <= LDR_4;/*if (opcode == 3'b100) state <= STR_1;
            else state <= LDR_4;*/

        LDR_4: state <= LDR_5;

        LDR_5: state <= Load_IR;

        STRx: state <= STRy;

        STRy: state <= STR_1;

        STR_1: state <= STR_2;

        STR_2: state <= STR_3;

        STR_3: state <= Load_IR;

        Halt: state <= Halt;

        default: state <= Reset;
      endcase
      end
    end

    always_comb begin

      casex(state)

        Reset: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b1, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b1, 1'b1, 1'bx, 1'bx};
        
        Fetch: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b1, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b1, 1'b0, 1'b0, 1'bx, 1'bx};
        
        Load_IR: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b1, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1};
        
        Fetch2: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b1, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0};
        
        Decode: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'bx, 1'b0, 1'b0, 1'bx, 1'b0};
          
        WriteImm8: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'b10, 2'b10, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'bx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx};
                
        StoreRn: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'b10, 2'bxx, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
               
        StoreRm: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'b00, 2'bxx, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
                       
        Add: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        
        Cmp: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        
        And: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        
        Mvn: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'bx, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};

        sh_Rm1: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} =  {1'b0, 2'b00, 2'bxx, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        
        sh_Rm2: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} =  {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        
        Write_final: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'b01, 2'b00, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'bx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx};

        Write_sh: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'b0, 2'b01, 2'b00, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'bx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx};

        LDR_1: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'b10, 2'bxx, 1'bx, 1'b1, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx};

        LDR_2: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'b0, 1'b0, 1'b1, 1'bx, 1'b0, 1'b1, 1'bx, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx};
        
        LDR_3: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'bx, 1'bx, 1'b1, 1'bx};

        LDR_4: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx};

        LDR_5: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'b01, 2'b11, 1'b1, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};

        STRx: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx, 1'b0, 1'b1, 1'bx, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx};
        
        STRy: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'b0, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'bx, 1'bx, 1'b1, 1'bx};
        
        STR_1: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'b01, 2'bxx, 1'bx, 1'b0, 1'b1, 1'bx, 1'bx, 1'b1, 1'bx, 1'bx, 1'b0, 1'bx, 1'bx, 1'b1, 1'bx};

        STR_2: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'b0, 1'b0, 1'b1, 1'bx, 1'b1, 1'b0, 1'bx, 1'b0, 1'bx, 1'bx, 1'b1, 1'bx};

        STR_3: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b1, 1'b0, 1'b1, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx};

        default: {waiting, reg_sel, wb_sel, w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, ram_w_en, sel_addr, load_pc, clear_pc, load_addr, load_ir} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'b0, 1'bx, 1'bx, 1'bx, 1'bx};


      endcase
    end

endmodule//: controller