# xSPI_8s_wk2530
This repository contains Verilog modules for a simple, custom SPI-like protocol called xSPI. It provides a complete controller (master) and slave system, both implemented with finite state machines. The design handles basic command, address, and data transactions for read and write operations, serving as a clear example for serial communication.
# xSPI Protocol 

This repository contains Verilog code for eXpanded Serial Peripheral Interface, referred to as **xSPI**. It includes a top-level module, a master controller, and a slave device, demonstrating a basic communication flow for sending commands, addresses, and transferring 64-bit data.

## Overview

The xSPI protocol is designed for communicating between a master (controller) and a slave device over a shared 8-bit IO bus. It supports:

  * 8-bit commands
  * 48-bit addresses
  * 64-bit data transfers (both read and write)

The core idea is to simplify SPI by using a single 8-bit bidirectional data bus instead of separate MOSI/MISO lines, making it more akin to a simplified Quad SPI (although only using 8 lines, not necessarily 4 pairs).

## Protocol Details

The communication is driven by the master (controller) which asserts `cs_n` (chip select) and `sck` (serial clock). Data is transferred byte by byte over the `io_bus`.

### Transaction Flow

A typical transaction follows these steps:

1.  **Idle**: Master and slave are in an idle state, `cs_n` is high.
2.  **Start Transaction**: The master asserts `start` (input to `xspi_sopi_controller`).
3.  **Command Phase**:
      * `cs_n` goes low.
      * The master drives the 8-bit `command` onto `io_bus`.
4.  **Address Phase**:
      * Following the command, the master drives the 48-bit `address` (6 bytes) onto `io_bus`, one byte per clock cycle.
5.  **Data Phase (Read or Write)**:
      * Based on the `command` received, the slave either expects write data or prepares to send read data.
      * **Write (Command `8'hA5`)**: The master drives 64-bit `wr_data` (8 bytes) onto `io_bus`. The slave receives and stores this data.
      * **Read (Command `8'hFF`)**: The master releases `io_bus` (`io_oe` goes low). The slave drives 64-bit data (8 bytes) from its internal memory onto `io_bus` for the master to read.
6.  **Finish/Done**:
      * After the data transfer, the master de-asserts `cs_n` (goes high).
      * The `done` flag on the master is asserted, and the `ready` flag on the slave is asserted.
      * Both return to their idle states, awaiting the next transaction.

### Commands

Currently, two commands are supported by the `xspi_sopi_slave`:

  * `8'hA5`: **Write Data** - The master sends 64 bits of data to be written to the slave's internal memory at the specified address.
  * `8'hFF`: **Read Data** - The master requests 64 bits of data from the slave's internal memory at the specified address. The slave then drives this data onto the bus.

Any other command will cause the transaction to terminate after the address phase, and no data transfer will occur.

## Module Descriptions

### `xspi_top`

The top-level module that instantiates and connects the `xspi_sopi_controller` (master) and `xspi_sopi_slave`. It handles the bidirectional `io_bus` by using simple combinational logic to select which module's output drives the bus based on their `io_oe` (output enable) signals.

**Parameters:**

  * `clk`: System clock
  * `rst_n`: Active-low reset
  * `start`: Input to trigger a transaction from the master
  * `command [7:0]`: 8-bit command to send
  * `address [47:0]`: 48-bit address for the transaction
  * `wr_data [63:0]`: 64-bit data to write during a write transaction
  * `rd_data [63:0]`: 64-bit data read from the slave during a read transaction
  * `done`: Output flag from the master, indicates transaction completion
  * `ready`: Output flag from the slave, indicates slave is ready for a new transaction

### `xspi_sopi_controller`

This module acts as the master in the xSPI protocol. It generates the `cs_n` and `sck` signals, drives data onto the `io_bus` for commands, addresses, and write data, and receives data during read operations. It uses a finite state machine (FSM) to manage the transaction flow.

**States:**

  * `STATE_IDLE`: Waiting for `start` signal.
  * `STATE_CMD`: Sending the 8-bit command.
  * `STATE_ADDR`: Sending the 48-bit address (6 bytes).
  * `STATE_WR_DATA`: Sending 64-bit write data (8 bytes).
  * `STATE_RD_DATA`: Receiving 64-bit read data (8 bytes).
  * `STATE_FINISH`: De-asserting `cs_n` and signaling `done`.

### `xspi_sopi_slave`

This module acts as the slave in the xSPI protocol. It responds to the `cs_n` and `sck` signals, receives commands and addresses, and performs either a write or read operation based on the command. It includes a simple 64-bit internal memory for demonstration purposes.

**States:**

  * `STATE_IDLE`: Initial state, or waiting for `cs_n` to go low.
  * `STATE_CMD`: Receiving the 8-bit command.
  * `STATE_ADDR`: Receiving the 48-bit address (6 bytes).
  * `STATE_WR_DATA`: Receiving 64-bit write data (8 bytes) and storing it in memory.
  * `STATE_RD_DATA`: Sending 64-bit read data (8 bytes) from memory.
  * `STATE_DONE`: Transaction complete, `ready` is asserted, waiting for `cs_n` to go high.

**Note on Slave Clocking**: The slave's state machine transitions are on the **negedge clk** (negative edge of the clock) for compatibility with tools like Verilator for behavioral simulation.

## Usage

To use this design, instantiate the `xspi_top` module in your testbench or larger system. Provide the necessary clock and reset signals, along with the command, address, and data inputs to initiate transactions. Monitor the `done` and `rd_data` outputs.

## Simulation

You can simulate this design using any Verilog simulator (e.g., Icarus Verilog, Modelsim, VCS, Verilator). A typical simulation flow would involve:

1.  Creating a testbench (`tb_xspi_top.v`) that instantiates `xspi_top`.
2.  Generating clock and reset signals within the testbench.
3.  Driving the `start`, `command`, `address`, and `wr_data` inputs to simulate master transactions.
4.  Monitoring the `rd_data`, `done`, and `ready` outputs to verify correct operation.

-----
