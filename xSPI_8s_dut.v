//! @file xSPI.v
//! @brief Top-level and submodules for a simple SPI-like protocol (xSPI) with controller and slave.

/**
 * @module xspi_top
 * @brief Top-level module connecting xSPI controller and slave via a shared IO bus.
 * @param clk     System clock
 * @param rst_n   Active-low reset
 * @param start   Start transaction
 * @param command 8-bit command
 * @param address 48-bit address
 * @param wr_data 64-bit data to write
 * @param rd_data 64-bit data read
 * @param done    Transaction done flag
 * @param ready   Slave ready flag
 */
module xspi_top (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset
    input  wire        start,    //!< Start transaction
    //input  wire        rw,
    input  wire [7:0]  command,  //!< 8-bit command
    input  wire [47:0] address,  //!< 48-bit address
    input  wire [63:0] wr_data,  //!< 64-bit data to write
    output wire [63:0] rd_data,  //!< 64-bit data read
    output wire        done,     //!< Transaction done flag
    output wire        ready     //!< Slave ready flag
);

    // Internal bus and control signals
    wire        cs_n;           //!< Chip select (active low)
    wire        sck;            //!< SPI clock
    wire [7:0]  master_io_out;  //!< Controller output to bus
    wire [7:0]  slave_io_out;   //!< Slave output to bus
    wire        master_io_oe;   //!< Controller output enable
    wire        slave_io_oe;    //!< Slave output enable
    wire [7:0]  io_bus;         //!< Shared IO bus

    // Simplified IO bus - no tri-state logic
    assign io_bus = master_io_oe ? master_io_out : 
                    slave_io_oe  ? slave_io_out  : 8'h00;

    /**
     * @brief xSPI controller (master)
     */
    xspi_sopi_controller master (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(master_io_out),
        .io_in(io_bus),
        .io_oe(master_io_oe),
        .start(start),
        //.rw(rw),
        .command(command),
        .address(address),
        .wr_data(wr_data),
        .rd_data(rd_data),
        .done(done)
    );

    /**
     * @brief xSPI slave
     */
    xspi_sopi_slave slave (
        .clk(clk),
        .rst_n(rst_n),
        .cs_n(cs_n),
        .sck(sck),
        .io_out(slave_io_out),
        .io_in(io_bus),
        .io_oe(slave_io_oe),
        .ready(ready)
    );

endmodule

/**
 * @module xspi_sopi_controller
 * @brief xSPI controller (master) for command/address/data transfer.
 * @param clk      System clock
 * @param rst_n    Active-low reset
 * @param cs_n     Chip select (active low)
 * @param sck      SPI clock
 * @param io_out   Output to IO bus
 * @param io_in    Input from IO bus
 * @param io_oe    Output enable for IO bus
 * @param start    Start transaction
 * @param command  8-bit command
 * @param address  48-bit address
 * @param wr_data  64-bit data to write
 * @param rd_data  64-bit data read
 * @param done     Transaction done flag
 */
module xspi_sopi_controller (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset

    // SPI signals - separated input/output
    output reg         cs_n,     //!< Chip select (active low)
    output reg         sck,      //!< SPI clock
    output reg [7:0]   io_out,   //!< Output to IO bus
    input  wire [7:0]  io_in,    //!< Input from IO bus
    output reg         io_oe,    //!< Output enable for IO bus

    // Control signals
    input  wire        start,    //!< Start transaction
   // input  wire        rw,         // 0 = write, 1 = read
    input  wire [7:0]  command,  //!< 8-bit command
    input  wire [47:0] address,  //!< 48-bit address
    input  wire [63:0] wr_data,  //!< 64-bit data to write

    output reg [63:0]  rd_data,  //!< 64-bit data read
    output reg         done      //!< Transaction done flag
);

    // === FSM States ===
    reg [2:0] state;         //!< Current FSM state
    reg [2:0] next_state;    //!< Next FSM state
    localparam STATE_IDLE      = 3'd0; //!< Idle state
    localparam STATE_CMD       = 3'd1; //!< Send command
    localparam STATE_ADDR      = 3'd2; //!< Send address
    localparam STATE_WR_DATA   = 3'd3; //!< Write data
    localparam STATE_RD_DATA   = 3'd4; //!< Read data
    localparam STATE_FINISH    = 3'd5; //!< Finish/cleanup

    // === Counters and Buffers ===
    reg [3:0] byte_cnt;      //!< Byte counter for address/data
    reg [63:0] rdata_buf;    //!< Buffer for read data

    /**
     * @brief FSM sequential logic: state update
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= STATE_IDLE;
        else
            state <= next_state;
    end

    /**
     * @brief FSM combinational logic: next state logic
     */
    always @(*) begin
        next_state = state;
        case (state)
            STATE_IDLE:
                if (start)
                    next_state = STATE_CMD;
            STATE_CMD:
                //if (byte_cnt == 1)
                    next_state = STATE_ADDR;
            STATE_ADDR:
              if (byte_cnt == 6) 
                next_state = (command == 8'hFF) ? STATE_RD_DATA : (command == 8'hA5) ? STATE_WR_DATA : STATE_FINISH; 
            STATE_WR_DATA:
              if (byte_cnt == 14)
                    next_state = STATE_FINISH;
            STATE_RD_DATA:
              if (byte_cnt == 15)
                    next_state = STATE_FINISH;
            STATE_FINISH:
                next_state = STATE_IDLE;
            default:
                next_state = STATE_IDLE;
        endcase
    end

    /**
     * @brief Main operation: drive SPI signals and manage data transfer
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n     <= 1'b1;
            sck      <= 1'b0;
            io_out   <= 8'h00;
            io_oe    <= 1'b0;
            done     <= 1'b0;
            rd_data  <= 64'h0;
            rdata_buf <= 64'h0;
            byte_cnt <= 4'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    cs_n     <= 1'b1;
                    sck      <= 1'b0;
                    io_oe    <= 1'b0;
                    done     <= 1'b0;
                    byte_cnt <= 4'd0;
                end

                STATE_CMD: begin
                    cs_n     <= 1'b0;
                    io_oe    <= 1'b1;
                    io_out   <= command;
                    byte_cnt <= byte_cnt + 1;
                end

                STATE_ADDR: begin
                  io_oe    <= 1'b1;
                  case (byte_cnt)
                        1: io_out <= address[47:40];
                        2: io_out <= address[39:32];
                        3: io_out <= address[31:24];
                        4: io_out <= address[23:16];
                        5: io_out <= address[15:8];
                        6: io_out <= address[7:0];
                  endcase
                    byte_cnt <= byte_cnt + 1;
                end

                STATE_WR_DATA: begin
                  io_oe    <= 1'b1;
                  case (byte_cnt)
                    7: io_out <= wr_data[63:56];
                        8: io_out <= wr_data[55:48];
                        9: io_out <= wr_data[47:40];
                        10: io_out <= wr_data[39:32];
                       11: io_out <= wr_data[31:24];
                       12: io_out <= wr_data[23:16];
                       13: io_out <= wr_data[15:8];
                       14: io_out <= wr_data[7:0];
                  endcase
                    byte_cnt <= byte_cnt + 1;
                end

                STATE_RD_DATA: begin
                    io_oe <= 1'b0;
                  case (byte_cnt)
                        8: rdata_buf[63:56] <= io_in;
                        9: rdata_buf[55:48] <= io_in;
                        10: rdata_buf[47:40] <= io_in;
                        11: rdata_buf[39:32] <= io_in;
                        12: rdata_buf[31:24] <= io_in;
                        13: rdata_buf[23:16] <= io_in;
                        14: rdata_buf[15:8]  <= io_in;
                        15: rdata_buf[7:0]   <= io_in;
                  endcase
                    byte_cnt <= byte_cnt + 1;
                end

                STATE_FINISH: begin
                    cs_n    <= 1'b1;
                    io_oe   <= 1'b0;
                    done    <= 1'b1;
                    rd_data <= rdata_buf;
                end
            endcase
        end
    end
endmodule

/**
 * @module xspi_sopi_slave
 * @brief xSPI slave for command/address/data transfer and simple memory.
 * @param clk      System clock
 * @param rst_n    Active-low reset
 * @param cs_n     Chip select (active low)
 * @param sck      SPI clock
 * @param io_out   Output to IO bus
 * @param io_in    Input from IO bus
 * @param io_oe    Output enable for IO bus
 * @param ready    Ready flag (debug/status)
 */
module xspi_sopi_slave (
    input  wire        clk,      //!< System clock
    input  wire        rst_n,    //!< Active-low reset

    input  wire        cs_n,     //!< Chip select (active low)
    input  wire        sck,      //!< SPI clock
    output reg [7:0]   io_out,   //!< Output to IO bus
    input  wire [7:0]  io_in,    //!< Input from IO bus
    output reg         io_oe,    //!< Output enable for IO bus

    output reg         ready     //!< Ready flag (debug/status)
);

    // === FSM States ===
    reg [2:0] state;         //!< Current FSM state
    localparam STATE_IDLE      = 3'd0; //!< Idle state
    localparam STATE_CMD       = 3'd1; //!< Receive command
    localparam STATE_ADDR      = 3'd2; //!< Receive address
    localparam STATE_WR_DATA   = 3'd3; //!< Receive write data
    localparam STATE_RD_DATA   = 3'd4; //!< Send read data
    localparam STATE_DONE      = 3'd5; //!< Done state

    reg [3:0] byte_cnt;      //!< Byte counter for address/data

    // === Internal Buffers ===
    reg [7:0]  command_reg;  //!< Latched command
    reg [47:0] addr_reg;     //!< Latched address
    reg [63:0] data_reg;     //!< Data buffer

    // === Simple Memory (64 x 64-bit = 512 bytes) ===
    reg [63:0] mem ;   //!< Simple memory array

    /**
     * @brief State machine for slave operation (posedge clk for Verilator compatibility)
     */
  always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= STATE_IDLE;
            io_out     <= 8'h00;
            io_oe      <= 1'b0;
            byte_cnt   <= 4'd0;
            ready      <= 1'b0;
            command_reg <= 8'h00;
            addr_reg   <= 48'h0;
            data_reg   <= 64'h0;
        end else begin
            if (cs_n) begin
                // CS# high: reset state machine
                state    <= STATE_CMD;
                io_oe    <= 1'b0;
                byte_cnt <= 0;
                ready    <= 1'b0;
            end else begin
                case (state)
                    STATE_IDLE: begin
                        byte_cnt <= 0;
                        state    <= STATE_CMD;
                    end

                    STATE_CMD: begin
                        command_reg <= io_in;
                        byte_cnt    <= 1;
                        state       <= STATE_ADDR;
                    end

                    STATE_ADDR: begin
                        case (byte_cnt)
                            1: addr_reg[47:40] <= io_in;
                            2: addr_reg[39:32] <= io_in;
                            3: addr_reg[31:24] <= io_in;
                            4: addr_reg[23:16] <= io_in;
                            5: addr_reg[15:8]  <= io_in;
                            6: addr_reg[7:0]   <= io_in;
                        endcase
                        byte_cnt <= byte_cnt + 1;
                        if (byte_cnt == 6)
                            state <= (command_reg == 8'hFF) ? STATE_RD_DATA :
                                     (command_reg == 8'hA5) ? STATE_WR_DATA : STATE_DONE;
                    end

                    STATE_WR_DATA: begin
                        case (byte_cnt)
                            7: data_reg[63:56] <= io_in;
                            8: data_reg[55:48] <= io_in;
                            9: data_reg[47:40] <= io_in;
                           10: data_reg[39:32] <= io_in;
                           11: data_reg[31:24] <= io_in;
                           12: data_reg[23:16] <= io_in;
                           13: data_reg[15:8]  <= io_in;
                           14: data_reg[7:0]   <= io_in;
                        endcase
                        byte_cnt <= byte_cnt + 1;
                      if (byte_cnt == 14) begin
                            mem <= {io_in, data_reg[55:0]}; // last byte completes 64-bit
                            state <= STATE_DONE;
                        end
                    end

                    STATE_RD_DATA: begin
                      if (byte_cnt == 6)
                        data_reg <= mem;
                        io_oe  <= 1'b1;
                        case (byte_cnt)
                            7: io_out <= data_reg[63:56];
                            8: io_out <= data_reg[55:48];
                            9: io_out <= data_reg[47:40];
                           10: io_out <= data_reg[39:32];
                           11: io_out <= data_reg[31:24];
                           12: io_out <= data_reg[23:16];
                           13: io_out <= data_reg[15:8];
                           14: io_out <= data_reg[7:0];
                        endcase
                        byte_cnt <= byte_cnt + 1;
                      if (byte_cnt == 14)
                            state <= STATE_DONE;
                    end

                    STATE_DONE: begin
                        io_oe <= 1'b0;
                        ready <= 1'b1;
                        // Wait for CS# to go high
                    end

                    default: state <= STATE_IDLE;
                endcase
            end
        end
    end
endmodule
