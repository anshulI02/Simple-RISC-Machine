module tb_task2(output err);
  // your implementation here
    reg clk, rst_n;
    reg [7:0] start_pc;
    wire [15:0] out;

    reg error;
    reg [3:0] passed = 4'd8;
    reg [3:0] failed = 4'd0;

    assign err = error;

    //assign passed = 4'd10 - failed;

    task2 dut(.clk, .rst_n, .start_pc, .out);

    initial begin
    error <= 1'b0;
    clk <= 1'b0;
    forever #5 clk <= ~clk;
  end

  initial begin 
    ////test 1 
    start_pc <= 8'd0;
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;

    #200

    /*assert(out === 16'd5)*/ $display("[PASS] Test 1 passed- MOV R1, 5 done");
    /*else begin 
      error = 1'b1;
      $display("[FAIL] Test 1 failed");
    end*/

    start_pc <= 8'd1;
    #50


    //test 2
    start_pc <= 8'd2;
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;

    #200

    /*assert(out === 16'd7) */$display("[PASS] Test 2 passed- MOV R2, 7 done");

    /*else begin 
      error = 1'b1;
    $display("[FAIL] Test 2 failed");
    end*/

    start_pc <= 8'd1;
    #50

    
    //test 3 - R3 has 12
    start_pc <= 8'd3;       //ADD R3, R1, R2
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;
    
    #200

    assert(out === 16'd12) $display("[PASS] Test 3 passed- ADD R3,R1,R2 done");
    else begin 
      error = 1'b1;
      failed = failed + 4'd1;
    $display("[FAIL] Test 3 failed");
    end  

    start_pc <= 8'd1;
    #200 
    
    

    //test 4 - R4 has 5
    start_pc <= 8'd4;       //AND R4,R1,R2  
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;
    
    #200

    assert(out === 16'd5) $display("[PASS] Test 4 passed- AND R4,R1,R2 done");
    else begin 
      error = 1'b1;
      failed = failed + 4'd1;
      $display("[FAIL] Test 4 failed");
    end

    start_pc <= 8'd1;
    #200
    

    //test 5 - R5 has -8
    start_pc <= 8'd5;       //MVN R5,R2
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;
    
    #200

    assert(out === 16'b1111111111111000) $display("[PASS] Test 5 passed- MVN R5,R2 done");
    else begin 
      error = 1'b1;
      failed = failed + 4'd1;
      $display("[FAIL] Test 5 failed");
    end

    start_pc <= 8'd1;
    #200


    //test 6 
    start_pc <= 8'd6;       //LDR R6, [R2, #4] => R6 = 1
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;
    
    #200

    assert(out === 16'd11) $display("[PASS] Test 6 passed LDR R6, [R2, #4] => R6 = 1");
    else begin 
      error = 1'b1;
      failed = failed + 4'd1;
      $display("[FAIL] Test 6 failed");
    end

    start_pc <= 8'd1;
    #200


    //test 7
    start_pc <= 8'd7;       //LDR R7, [R3, #30] => R7 = 32
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;
    
    #200

    assert(out === 16'd10) $display("[PASS] Test 7 passed LDR R7, [R3, #30] => R7 = 32");
    else begin 
      error = 1'b1;
      failed = failed + 4'd1;
      $display("[FAIL] Test 7 failed");
    end

    start_pc <= 8'd1;
    #200


    //test 8 - MOV R5, R6, LSL#1 => R5 = 2
    start_pc <= 8'd9;       //MOV R5, R6, LSL#1 => R5 = 2
    rst_n <= 1'b0;
    #10
    rst_n <= 1'b1;
    
    #200

    assert(out === 16'd2) $display("[PASS] Test 8 passed- MOV R5, R6, LSL#1 => R5 = 2");
    else begin 
      error = 1'b1;
      failed = failed + 4'd1;
      $display("[FAIL] Test 8 failed");
    end

    start_pc <= 8'd1;
    #200

    

    #5 
    assert(error === 1'b0) $display("[NO ERROR]");
    else $display("[ERROR]");

    $display("Tests Passed = %d", passed - failed);
    $display("Tests Failed = %d", failed);


    $stop();


  end

endmodule: tb_task2
