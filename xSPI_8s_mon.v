`timescale 1ns/1ps

module xspi_monitor (
    input  wire        clk,
    input  wire        done,
    input  wire        ready,
    input  wire [63:0] rd_data
);

    reg [1:0] op_count = 0;

    always @(posedge clk) begin
        if (done) begin
            op_count <= op_count + 1;
            if (op_count == 0) begin
                $display("[WRITE] Write operation completed.");
            end else if (op_count == 1) begin
                $display("[READ] Read operation completed. Data = %h", rd_data);
                if (rd_data == 64'h1122334455667788) begin
                    $display("✅ PASS: Read data matches written data.");
                end else begin
                    $display("❌ FAIL: Read data mismatch! Got %h", rd_data);
                end
            end
        end

        if (ready) begin
            $display("✅ Slave ready at time %0t", $time);
        end
    end
endmodule
