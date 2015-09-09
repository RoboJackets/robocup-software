`ifndef _SPI_SLAVE_
`define _SPI_SLAVE_

module SPI_Slave #(
  parameter       CPOL = ( 0 ),
                  CPHA = ( 0 )
  )(
  input           clk,
  input           ncs,
  input           mosi,
  output          miso,
  input           sck,
  output          done,
  input   [7:0]   din,
  output  [7:0]   dout
);
   
  reg ncs_q,
      sck_q,            sck_old_q,
      mosi_dp,          mosi_dn,    mosi_q,
      done_dp,          done_dn,    done_q,
      miso_dp,          miso_dn,    miso_q;

  reg [7:0] data_dp,    data_dn,    data_q,
            dout_dp,    dout_dn,    dout_q;

  reg [2:0] bit_ct_dp,  bit_ct_dn,  bit_ct_q;

  wire  rising_edge  = !sck_old_q  &&   sck_q,
        falling_edge =  sck_old_q  &&  !sck_q;
   

  assign miso = miso_q;
  assign done = done_q;
  assign dout = dout_q;


   /*
  always @(*) begin
    ss_d = ss;
    sck_d = sck;
    sck_old_d = sck_q;
    
    mosi_dp = mosi;
    miso_dp = miso_q;
    done_dp = 0;
    bit_ct_dp = bit_ct_q;
    dout_dp = dout_q;

    mosi_dn = mosi;
    miso_dn = miso_q;
    done_dn = 0;
    bit_ct_dn = bit_ct_q;
    dout_dn = dout_q;
  end
  */

  always @( negedge clk ) begin
      if ( falling_edge ) begin
          data_dn <= {data_dn[6:0], mosi_q};       // read data in and shift
          bit_ct_dn <= bit_ct_q + 1;           // increment the bit counter

          if (bit_ct_dn == 3'b111) begin         // if we are on the last bit
            dout_dn <= {data_q[6:0], mosi_q};     // output the byte
            done_dn <= 1;                      // set transfer done flag
            data_dn <= din;                       // read in new byte
          end

          miso_dn <= data_dn[7];                   // output MSB for other sck polarity
      end else if ( rising_edge ) begin
          data_dp <= {data_dp[6:0], mosi_q};       // read data in and shift
          bit_ct_dp <= bit_ct_q + 1;           // increment the bit counter

          if (bit_ct_dp == 3'b111) begin         // if we are on the last bit
            dout_dp <= {data_q[6:0], mosi_q};     // output the byte
            done_dp <= 1;                      // set transfer done flag
            data_dp <= din;                       // read in new byte
          end

          miso_dp <= data_dp[7];                   // output MSB for other sck polarity
      end
  end

always @( posedge clk ) begin
  if (ncs) begin
      // bit_ct_dp <= 0;                 // reset bit counter
      // data_dp <= din;                 // read in data
      // miso_dp <= data_q[7];           // output MSB

      // bit_ct_dn <= 0;                 // reset bit counter
      // data_dn <= din;                 // read in data
      // miso_dn <= data_q[7];           // output MSB
      done_q <= 0;
      //bit_ct_q <= bit_ct_dn;
      //dout_q <= dout_dn;
      miso_q <= 0;
      mosi_q <= 0;
      data_q <= 0;
      data_dp <= din;
      data_dn <= din;
  end else begin

    data_q <= (CPOL ^ CPHA) == 1 ? data_dn : data_dp; 

    if (CPOL ^ CPHA) begin
      done_q <= done_dn;
      bit_ct_q <= bit_ct_dn;
      dout_q <= dout_dn;
      miso_q <= miso_dp;
      mosi_q <= mosi_dn;
      // data_q <= data_dn;
    end else begin
      done_q <= done_dp;
      bit_ct_q <= bit_ct_dp;
      dout_q <= dout_dp;
      miso_q <= miso_dn;
      mosi_q <= mosi_dp;
      // data_q <= data_dp;
    end
  end

    sck_q <= sck;
    ncs_q <= ncs;
    sck_old_q <= sck_q;
  end
   
endmodule

`endif
