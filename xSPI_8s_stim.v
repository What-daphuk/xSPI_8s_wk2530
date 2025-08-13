`timescale 1ns/1ps

module xspi_stimulus (
    output reg        clk,
    output reg        rst_n,
    output reg        start,
    output reg [7:0]  command,
    output reg [47:0] address,
    output reg [63:0] wr_data
);

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

    initial begin
        rst_n = 0;
        start = 0;
        command = 0;
        address = 0;
        wr_data = 0;

        #20;
        rst_n = 1;
        #20;

        // --- WRITE ---
        @(posedge clk);
        command = 8'hA5;
        address = 48'h6655443322AB;
        wr_data = 64'h1122334455667788;
        start = 1;

        @(posedge clk);
        start = 0;

        // Wait for some time before issuing READ
        #200;

        // --- READ ---
        @(posedge clk);
        command = 8'hFF;
        address = 48'h6655443322AB;
        wr_data = 64'h0;
        start = 1;

        @(posedge clk);
        start = 0;
    end
initial begin
#900;
$finish;
end
endmodule
