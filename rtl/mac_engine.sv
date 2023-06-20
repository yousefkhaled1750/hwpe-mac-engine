/*
 * mac_engine.sv
 * Francesco Conti <f.conti@unibo.it>
 *
 * Copyright (C) 2018-2022 ETH Zurich, University of Bologna
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * The architecture that follows is relatively straightforward; it supports two modes:
 *  - in 'simple_mult' mode, the a_i and b_i streams feed the 32b x 32b multiplier (mult).
 *    The output of the multiplier (64b) is registered in a pipeline stage
 *    (r_mult), which is then shifted by ctrl_i.shift to the right and streamed out as d_o.
 *    There is no control local to the module except for handshakes.
 *  - in 'scalar_prod' mode, the c_i stream is first shifted left by ctrl_i.shift, extended
 *    to 64b and saved in r_acc. Then, the a_i and b_i streams feed the 32b x 32b multiplier
 *    (mult) for ctrl_i.len cycles, controlled by a local counter. The output of mult is 
 *    registered in a pipeline stage (r_mult), whose value is used as input to an accumulator
 *    (r_acc) -- the one which was inited by the shifted value of c_i. At the end of the
 *    ctrl_i.len cycles, the output of r_acc is shifted back to the right by ctrl_i.shift
 *    bits and streamed out as d_o.
 */

import mac_package::*;
 
// all the changes we are going to implement:
//  1. remove b_i, c_i -> requires to remove all the propagates streams upward (stream b,c, tcdm b,c)
//     and change the number of ports from 4 into 2




module mac_engine
(
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // input a stream
  hwpe_stream_intf_stream.sink   a_i,
  // input b stream
  //hwpe_stream_intf_stream.sink   b_i,
  //// input c stream
  //hwpe_stream_intf_stream.sink   c_i,
  // output d stream
  hwpe_stream_intf_stream.source d_o,
  // control channel
  input  ctrl_engine_t           ctrl_i,
  output flags_engine_t          flags_o
);

  // logic unsigned [$clog2(MAC_CNT_LEN):0] cnt;
  // logic unsigned [$clog2(MAC_CNT_LEN):0] r_cnt;
  logic unsigned [$clog2(MAC_CNT_LEN):0] cnt_out;
  logic unsigned [$clog2(MAC_CNT_LEN):0] r_cnt_out;
  


  logic farrow_en;
  logic farrow_valid;
  logic [32-1:0] farrow_data_in;
  logic [32-1:0] farrow_data_out;
  logic [32-1:0] farrow_mu;



  // logic                                   mat_valid; //a pulse for a clock cycle 
  // logic    signed [24 - 1 : 0]    b11, b12,b13;
  // logic    signed [24 - 1 : 0]    b21, b22,b23;
  // logic    signed [24 - 1 : 0]    b31, b32,b33;
  
  // logic    signed [24 - 1 : 0]    r_b11, r_b12, r_b13;
  // logic    signed [24 - 1 : 0]    r_b21, r_b22, r_b23;
  // logic    signed [24 - 1 : 0]    r_b31, r_b32, r_b33;
  


  // logic signed [63:0] c_shifted;
  // logic signed [63:0] mult;
  // logic signed [63:0] r_mult;
  // logic               r_mult_valid;
  // logic               r_mult_ready;
  // logic signed [64+$clog2(MAC_CNT_LEN)-1:0] r_acc;
  // logic                                     r_acc_valid;
  // logic                                     r_acc_ready;
  // logic signed [64+$clog2(MAC_CNT_LEN)-1:0] d_nonshifted;
  // logic                                     d_nonshifted_valid;

  // A design choice of this accelerator is that at the interface of modules only a few categories
  // of signals are exposed:
  //  - global signals (clk, rst_n)
  //  - HWPE-Stream or TCDM interfaces (a_i, ...)
  //  - a control packed struct (ctrl_i) and a state packed struct (flags_o)
  // The flags_o packed struct encapsulates all of the information about the internal state
  // of the module that must be exposed to the controller, and the ctrl_i all the control
  // information necessary for configuring the current module. In this way, it is possible to
  // make significant changes to the control interface (which can typically propagate through
  // a big hierarchy of modules) without manually modifying the interface in all modules; it
  // is sufficient to change the packed struct definition in the package where it is defined.
  // Packed structs are essentially bit vectors where bit fields have a name, and as such
  // are easily synthesizable and much more readable than Verilog-2001-ish code.

  // Take `c_i` and shift it by `ctrl_i.shift` bits to the left in a sign-preserving way.
  // The fixed-point representation of the input and output streams is not necessarily aligned
  // with that of the accumulator `r_acc`, which accounts for the `a_i*b_i` product.
  // Since `c_i` is used only to preload the accumulator, `c_shifted` is brought to the same
  // fixed-point format.
  //always_comb
  //begin : shift_c
  //  c_shifted = $signed(c_i.data <<< ctrl_i.shift);
  //end

  // Signed product between `a_i` and `b_i`. Notice this is combinational logic, and the
  // synthesis tool will implement it with an appropriate design from its synthetic library,
  // such as a Wallace-tree Booth multiplier.
  //always_comb
  //begin : mult_a_X_b
  //  mult = $signed(a_i.data) * $signed(b_i.data);
  //end

  // The output of the product is registered, introducing one cycle of latency. 
  // This pipeline register not only cuts combinational paths from `a_i` and `b_i`
  // to the adder after `mult`, but it is also design to preserve the correct data
  // flow by enabling backpressure control through a valid/ready handshake.
  // The register also includes an explicit soft clear (used to initialize the accelerator
  // via a register-mapped access without resetting the system) and an enable signal,
  // used for global clock gating.
  //always_ff @(posedge clk_i or negedge rst_ni)
  //begin : mult_pipe_data
  //  if(~rst_ni) begin
  //    r_mult <= '0;
  //  end
  //  else if (ctrl_i.clear) begin
  //    r_mult <= '0;
  //  end
  //  else if (ctrl_i.enable) begin
  //    // `r_mult` value is updated if there is a valid handshake at both its inputs;
  //    // in all other cases it is kept constant.
  //    if (a_i.valid & b_i.valid & a_i.ready & b_i.ready) begin
  //      r_mult <= mult;
  //    end
  //  end
  //end

  // This calculates the `valid` signal associated with `r_mult`. In this case, we
  // chose to propagate this signal explicitly through all pipeline registers in the
  // datapath to showcase explicitly how this can be done (in other accelerators,
  // one could manage validity in a less sophisticated way, e.g., with a counter).
  // In detail, `r_mult` is valid when both `a_i` and `b_i` are valid, with one
  // cycle of delay. The validity is evaluated only in two conditions:
  //  1) when a valid handshake happens at the output (`r_mult` valid & ready)
  //  2) when the inputs are known to be valid
  // Of course, by construction `r_mult_valid` can transition from 1 to 0 only in
  // condition 1), that is, following a valid handshake at the output.
  //always_ff @(posedge clk_i or negedge rst_ni)
  //begin : mult_pipe_valid
  //  if(~rst_ni) begin
  //    r_mult_valid <= '0;
  //  end
  //  else if (ctrl_i.clear) begin
  //    r_mult_valid <= '0;
  //  end
  //  else if (ctrl_i.enable) begin
  //    // r_mult_valid is re-evaluated after a valid handshake or in transition to 1
  //    if ((a_i.valid & b_i.valid) | (r_mult_valid & r_mult_ready)) begin
  //      r_mult_valid <= a_i.valid & b_i.valid;
  //    end
  //  end
  //end

  // `r_acc` is the accumulator register of the MAC engine. In this case, we coded
  // the full functionality of the accumulator (combinational sum and sequential register)
  // inside the same `always_ff` block. The behavior of the accumulator register here
  // is entirely driven by the self-timed valid & ready handshakes:
  //  1) if there are both new valid multiplier values and `c_i`, it inits the `r_acc`
  //     register with their sum (with `c_i` left-shifted);
  //  2) if there is only a `c_i` valid, it inits the `r_acc` with its (shifted) value;
  //  3) it only multiplier values are valid & ready, it accumulates the current value
  //     of `r_acc` with `r_mult`.
  //always_ff @(posedge clk_i or negedge rst_ni)
  //begin : accumulator
  //  if(~rst_ni) begin
  //    r_acc <= '0;
  //  end
  //  else if (ctrl_i.clear) begin
  //    r_acc <= '0;
  //  end
  //  else if (ctrl_i.enable) begin
  //    // r_acc value is updated if there are both c_i and r_mult valid handshakes at its input
  //    if (r_mult_valid & r_mult_ready & c_i.valid & c_i.ready) begin
  //      r_acc <= $signed(c_shifted + r_mult);
  //    end
  //    // r_acc value is updated if there is a c_i valid handshake at its input
  //    else if (c_i.valid & c_i.ready) begin
  //      r_acc <= $signed(c_shifted);
  //    end
  //    // r_acc value is updated if there is a r_mult valid handshake at its input
  //    else if (r_mult_valid & r_mult_ready) begin
  //      r_acc <= $signed(r_acc + r_mult);
  //    end
  //  end
  //end

  // Differently to `r_mult`, the validity of `r_acc` depends on a full dot-product having
  // happened, controlled by comparing the `r_cnt` counter with the size `ctrl_i.len`.
  // The validity is evaluated when the length reaches this threshold and the `r_mult` has 
  // a valid handshake (as `r_mult` is the input from `r_acc`'s viewpoint). It is also
  // re-evaluated after a correct output transition.
  //always_ff @(posedge clk_i or negedge rst_ni)
  //begin : accumulator_valid
  //  if(~rst_ni) begin
  //    r_acc_valid <= '0;
  //  end
  //  else if (ctrl_i.clear) begin
  //    r_acc_valid <= '0;
  //  end
  //  else if (ctrl_i.enable) begin
  //    // r_acc_valid is re-evaluated after a valid handshake or in transition to 1
  //    if(((r_cnt == ctrl_i.len) & r_mult_valid & r_mult_ready) | (r_acc_valid & r_acc_ready)) begin
  //      r_acc_valid <= (r_cnt == ctrl_i.len);
  //    end
  //  end
  //end

  // Depending on the configured mode, the output can be taken directly from `r_mult`, or
  // from `r_acc`. In both cases, it is right-shifted before streaming it out.
  //always_comb
  //begin : d_nonshifted_comb
  //  if(ctrl_i.simple_mul) begin
  //    d_nonshifted       = $signed(r_mult);
  //    d_nonshifted_valid = r_mult_valid;
  //  end
  //  else begin
  //    d_nonshifted       = r_acc;
  //    d_nonshifted_valid = r_acc_valid;
  //  end
  //end

  // // Output is right-shifted into the correct QF representation. There is currently
  // // no support for rounding and for saturation/clipping.
  // always_comb
  // begin
  //   d_o.data  = $signed(d_nonshifted >>> ctrl_i.shift); // no saturation/clipping
  //   d_o.valid = ctrl_i.enable & d_nonshifted_valid;
  //   d_o.strb  = '1; // strb is always '1 --> all bytes are considered valid
  // end

  // The control counter is implemented directly inside this module; as the control is
  // minimal, it was not deemed convenient to move it to another submodule. For bigger
  // FSMs that is typically the most advantageous choice.
  // The counter counts `r_mult` valid outputs.
  // always_comb
  // begin
  //   cnt = r_cnt + 1;
  // end
  // // the input counter
  // always_ff @(posedge clk_i or negedge rst_ni)
  // begin
  //   if(~rst_ni) begin
  //     r_cnt <= '0;
  //   end
  //   else if(ctrl_i.clear) begin
  //     r_cnt <= '0;
  //   end
  //   else if(ctrl_i.enable) begin
  //     // The counter is updated
  //     //  1) at the start of operations
  //     //  2) when the count value is between 0 and `len` (excluded), and there is a valid `r_mult` handshake.
  //     //if ((ctrl_i.start == 1'b1) || ((r_cnt > 0) && (r_cnt < ctrl_i.len) && (r_mult_valid & r_mult_ready == 1'b1))) begin
  //     if ((ctrl_i.start == 1'b1) || ((r_cnt > 0) && (a_i.valid & a_i.ready == 1'b1))) begin
  //       r_cnt <= cnt;
  //     end
  //   end
  // end

  // the output counter
  // the purpose it to order the mat_inv outputs into d stream
  // using a mux
  // assign cnt_out = r_cnt_out + 1;
  // always_ff @(posedge clk_i or negedge rst_ni)
  // begin
  //   if(~rst_ni) begin
  //     r_cnt_out <= '0;
  //   end
  //   else if(ctrl_i.clear) begin
  //     r_cnt_out <= '0;
  //   end
  //   else if(ctrl_i.enable) begin
  //     // The counter is updated
  //     //  1) at the start of operations
  //     //  2) when the count value is between 0 and `len` (excluded), and there is a valid `r_mult` handshake.
  //     //if ((ctrl_i.start == 1'b1) || ((r_cnt > 0) && (r_cnt < ctrl_i.len) && (r_mult_valid & r_mult_ready == 1'b1))) begin
  //     if ((mat_valid) || ((r_cnt_out > 0) && (r_cnt_out < ctrl_i.len) && (d_o.valid & d_o.ready == 1'b1))) begin
  //       r_cnt_out <= cnt_out;
  //     end
  //   end
  // end

/************************* ************************************ *************************/
/************************* the engine flag signals after update *************************/
/************************* ************************************ *************************/
  // Export counter and valid accumulator to main HWPE control FSM.
  //assign flags_o.cnt = r_cnt;
  assign flags_o.cnt_out = r_cnt_out;
  //assign flags_o.acc_valid = r_acc_valid;
  //assign flags_o.mat_valid = (r_cnt_out == ctrl_i.len);   // -> needs to be added to fsm and the package

  // Ready signals have to be propagated backwards through pipeline stages (combinationally).
  // To avoid deadlocks, the following rules have to be followed:
  //  1) transition of ready CAN depend on the current state of valid
  //  2) transition of valid CANNOT depend on the current state of ready
  //  3) transition 1->0 of valid MUST depend on (previous) ready (i.e., once the valid goes
  //     to 1 it cannot go back to 0 until there is a valid handshake)
  // In the following:
  // R_valid & R_ready denominate the handshake at the *output* (Q port) of pipe register R

  // Output accepts new value from accumulator when the output is ready or `r_acc` is invalid
  // assign r_acc_ready  = d_o.ready | ~r_acc_valid;
  // Accumulator accepts new value from multiplier when
  //  1) output is ready or `r_mult` is invalid (if in simple multiplication mode)
  //  2) `r_acc` is ready or `r_mult` is invalid (if in scalar product mode)
  // assign r_mult_ready = (ctrl_i.simple_mul) ? d_o.ready   | ~r_mult_valid
  //                                           : r_acc_ready | ~r_mult_valid;
  // Multiplier accepts new value from `a_i` and `b_i` when `r_mult` is ready and both
  // `a_i` & `b_i` are valid, or when both `a_i` & `b_i` are invalid
  assign a_i.ready = 1; // a_i.ready is the one initiating the load so it has to be high by default
  //assign a_i.ready = (r_mult_ready & a_i.valid & b_i.valid) | (~a_i.valid & ~b_i.valid);
  //assign b_i.ready = (r_mult_ready & a_i.valid & b_i.valid) | (~a_i.valid & ~b_i.valid);
  // Multiplier accepts new value from `c_i` when `r_acc` is ready or `c_i` is invalid
  //assign c_i.ready    = r_acc_ready  | ~c_i.valid;

  // The following assertions help in getting the rules on ready & valid right.
  // They are copied from the general stream rules in hwpe_stream_interfaces.sv
  // and adapted to the internal `r_acc` and `r_mult` signals.
  // `ifndef SYNTHESIS
  // `ifndef VERILATOR
  //   // The data and strb can change their value 1) when valid is deasserted,
  //   // 2) in the cycle after a valid handshake, even if valid remains asserted.
  //   // In other words, valid data must remain on the interface until
  //   // a valid handshake has occurred.
  //   property r_acc_change_rule;
  //     @(posedge clk_i)
  //     ($past(r_acc_valid) & ~($past(r_acc_valid) & $past(r_acc_ready))) |-> (r_acc == $past(r_acc));
  //   endproperty;
  //   property r_mult_change_rule;
  //     @(posedge clk_i)
  //     ($past(r_mult_valid) & ~($past(r_mult_valid) & $past(r_mult_ready))) |-> (r_mult == $past(r_mult));
  //   endproperty;
    
  //   // The deassertion of valid (transition 1->0) can happen only in the cycle
  //   // after a valid handshake. In other words, valid data produced by a source
  //   // must be consumed on the sink side before valid is deasserted.
  //   property r_acc_valid_deassert_rule;
  //     @(posedge clk_i)
  //     ($past(r_acc_valid) & ~r_acc_valid) |-> $past(r_acc_valid) & $past(r_acc_ready);
  //   endproperty;
  //   property r_mult_valid_deassert_rule;
  //     @(posedge clk_i)
  //     ($past(r_mult_valid) & ~r_mult_valid) |-> $past(r_mult_valid) & $past(r_mult_ready);
  //   endproperty;

  //   R_ACC_VALUE_CHANGE:    assert property(r_acc_change_rule)
  //     else $fatal("ASSERTION FAILURE: R_ACC_VALUE_CHANGE", 1);

  //   R_ACC_VALID_DEASSERT:  assert property(r_acc_valid_deassert_rule)
  //     else $fatal("ASSERTION FAILURE R_ACC_VALID_DEASSERT", 1);

  //   R_MULT_VALUE_CHANGE:   assert property(r_mult_change_rule)
  //     else $fatal("ASSERTION FAILURE: R_MULT_VALUE_CHANGE", 1);

  //   R_MULT_VALID_DEASSERT: assert property(r_mult_valid_deassert_rule)
  //     else $fatal("ASSERTION FAILURE R_MULT_VALID_DEASSERT", 1);
  // `endif /* VERILATOR */
  // `endif /* SYNTHESIS */




  // always_ff @(posedge clk_i or negedge rst_ni)
  // begin : main_fsm_seq
  //   if(~rst_ni) begin
  //     r_b11 <= '0; r_b12 <= '0; r_b13 = '0;
  //     r_b21 <= '0; r_b22 <= '0; r_b23 = '0;
  //     r_b31 <= '0; r_b32 <= '0; r_b33 = '0;
  //   end
  //   else if(ctrl_i.clear) begin
  //     r_b11 <= '0; r_b12 <= '0; r_b13 = '0;
  //     r_b21 <= '0; r_b22 <= '0; r_b23 = '0;
  //     r_b31 <= '0; r_b32 <= '0; r_b33 = '0;
  //   end
  //   else begin
  //       if (mat_valid) begin
  //           r_b11 <= b11; r_b12 <= b12; r_b13 = b13;
  //           r_b21 <= b21; r_b22 <= b22; r_b23 = b23;
  //           r_b31 <= b31; r_b32 <= b32; r_b33 = b33;
  //       end
  //   end
  // end

  // logic signed [32:0]  data_out_full;
  // // serializing the outputs to d_o
  // always_comb
  // begin : blockName
  //   case (r_cnt_out)
  //     'd1:  begin
  //         data_out_full <= {{8{0}},r_b11};
  //       end
  //     'd2:  begin
  //         data_out_full <= {{8{0}},r_b12};
  //       end 
  //     'd3:  begin
  //         data_out_full <= {{8{0}},r_b13};
  //       end
  //     'd4:  begin
  //         data_out_full <= {{8{0}},r_b21};
  //       end
  //     'd5:  begin
  //         data_out_full <= {{8{0}},r_b22};
  //       end 
  //     'd6:  begin
  //         data_out_full <= {{8{0}},r_b23};
  //       end
  //     'd7:  begin
  //         data_out_full <= {{8{0}},r_b31};
  //       end
  //     'd8:  begin
  //         data_out_full <= {{8{0}},r_b32};
  //       end 
  //     'd9:  begin
  //         data_out_full <= {{8{0}},r_b33};
  //       end
  //     default: begin
  //         data_out_full <= '0;
  //     end
  //   endcase
  // end


  // Output is right-shifted into the correct QF representation. There is currently
  // no support for rounding and for saturation/clipping.
  // always_comb
  // begin
  //   d_o.data  = $signed(data_out_full); // no saturation/clipping
  //   d_o.valid = ctrl_i.enable & r_cnt_out > 0;
  //   d_o.strb  = '1; // strb is always '1 --> all bytes are considered valid
  // end

// the philosophy of streaming the data is by getting the stream a (serially)
// and by checking the valid-ready handshake there will be a counter and mux that 
// store each data into the equivalent storage element and after finish storing,
// the enable_in signal will be raised for a single cycle
// after finishing, the engine will be waiting for the data_valid_out signal
// which will store all the 9 outputs and then send them one by one into d_o
// with raising the d_o.valid flag



// mat_inv_top #(
//   .INT_BITS(7),
//   .FRACT_BITS(17),
//   .ITERATIONS(20)
// ) mat_inv_top_i (
//     .clk_i(clk_i),
//     .rst_ni(rst_ni),
//     .clear_i(ctrl_i.clear),
//     .cnt_i(r_cnt),
//     .a_i(a_i.monitor),
//     .mat_valid_o(mat_valid),
//     .*
// );


  assign cnt_out = r_cnt_out + 1;
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if(~rst_ni) begin
      r_cnt_out <= '0;
    end
    else if(ctrl_i.clear) begin
      r_cnt_out <= '0;
    end
    else if(ctrl_i.enable) begin
      // The counter is updated
      //  1) at the start of operations
      //  2) when the count value is between 0 and `len` (excluded), and there is a valid `r_mult` handshake.
      //if ((ctrl_i.start == 1'b1) || ((r_cnt > 0) && (r_cnt < ctrl_i.len) && (r_mult_valid & r_mult_ready == 1'b1))) begin
      if ((farrow_valid) && (r_cnt_out < ctrl_i.len) && (d_o.valid & d_o.ready == 1'b1)) begin
        r_cnt_out <= cnt_out;
      end
    end
  end



always_comb
  begin
    d_o.data  = $signed(farrow_data_out); // no saturation/clipping
    d_o.valid = farrow_valid;
    d_o.strb  = '1; // strb is always '1 --> all bytes are considered valid
  end



always_comb begin
  farrow_data_in = a_i.data;
  farrow_en = a_i.valid;
end





farrow_third_sopot farrow_i 
(
.clk(clk_i),
.rst_n(rst_ni),
.en(farrow_en),
.in_data(farrow_data_in),
.mu(ctrl_i.mu),
.data_out(farrow_data_out),
.valid(farrow_valid)
);

//module farrow_third_sopot
//     #(parameter DATA_WIDTH=32,TAPS=8,Q=16,INT=8 ,FRAC=8,latency=7)
//     (
// input clk,rst_n,en,
// input [DATA_WIDTH-1:0] in_data,
// input [DATA_WIDTH-1:0] mu,
// output [DATA_WIDTH-1:0] data_out,
// output reg valid 
//     );


endmodule // mac_engine
