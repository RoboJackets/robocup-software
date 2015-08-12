/**
 * Copyright (C) 2014 Brooksee, Inc. 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 **/

// Author:    Lane Brooks
// Date:      10/31/2009
// Desc: Implements a low level SPI slave interface. 
//
//       Runs directly off sclk, which may not work for synthesis
//
//       The clock polarity and phasing of this master is set via the
//       CPOL and CPHA inputs. See
//       http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
//       a description of these conventions.
//


module SPI_Slave
  #(parameter DATA_WIDTH=16,
    parameter INPUT_SAMPLE_AND_HOLD=1
    )
  (input CPOL, 
   input CPHA,
   
   input [DATA_WIDTH-1:0] datai,
   output [DATA_WIDTH-1:0] datao,
   
   output  dout,
   input din,
   input csb,
   input sclk
   );

   reg [7:0] countp, countn;
   reg [DATA_WIDTH-1:0] sro_p, sro_n, sri_p, sri_n;
   
   wire [DATA_WIDTH-1:0] sro_p1,sro_n1;

   assign sro_p1 = (INPUT_SAMPLE_AND_HOLD) ? sro_p : datai << countp;
   assign sro_n1 = (INPUT_SAMPLE_AND_HOLD) ? sro_n : datai << countn;

   assign dout = (CPOL ^ CPHA) ? sro_p1[DATA_WIDTH-1] : sro_n1[DATA_WIDTH-1];
   assign datao= (CPOL ^ CPHA) ? sri_n : sri_p;

  always @(posedge sclk or posedge csb) begin

      if(csb) begin
          sro_p <= datai;
          countp <= 0;

      end else begin
          if(INPUT_SAMPLE_AND_HOLD) begin
              sro_p <= sro_p << 1;
          end else begin
              countp <= countp + 1;
          end

          sri_p <= { sri_p[DATA_WIDTH-2:0], din };
      end
  end

   always @(negedge sclk or posedge csb) begin
      if(csb) begin
     sro_n <= datai;
     countn <= 0;
      end else begin
     if(INPUT_SAMPLE_AND_HOLD) begin
        sro_n <= sro_n << 1;
     end else begin
        countn <= countn + 1;
     end
     sri_n <= { sri_n[DATA_WIDTH-2:0], din };
      end
   end

endmodule
   
