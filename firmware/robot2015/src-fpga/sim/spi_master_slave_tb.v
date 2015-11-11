module tb();

   parameter DATA_WIDTH=4;
   parameter CLK_DIVIDER_WIDTH=4;
   
   reg clk, resetb;
   wire [CLK_DIVIDER_WIDTH-1:0] clk_divider = 4;
   
   reg  go;

   wire [3:0] busy, done, sclk, csb, din, dout;
   reg [DATA_WIDTH-1:0] datai, datai_prev;
   wire [DATA_WIDTH-1:0] master_datao0, master_datao1, master_datao2, master_datao3;
   wire [DATA_WIDTH-1:0] slave_data0, slave_data1, slave_data2, slave_data3;
   
   spi_master #(.DATA_WIDTH(DATA_WIDTH),
        .CLK_DIVIDER_WIDTH(CLK_DIVIDER_WIDTH))
     spi_master0
     (.clk(clk),
      .resetb(resetb),
      .CPOL(1'b0),
      .CPHA(1'b0),
      .clk_divider(clk_divider),
      .go(go),
      .datai(datai),
      .datao(master_datao0),
      .busy(busy[0]),
      .done(done[0]),
      .sclk(sclk[0]),
      .csb(csb[0]),
      .din(din[0]),
      .dout(dout[0])
      );
   
   spi_master #(.DATA_WIDTH(DATA_WIDTH),
        .CLK_DIVIDER_WIDTH(CLK_DIVIDER_WIDTH))
     spi_master1
     (.clk(clk),
      .resetb(resetb),
      .CPOL(1'b1),
      .CPHA(1'b0),
      .clk_divider(clk_divider),
      .go(go),
      .datai(datai),
      .datao(master_datao1),
      .busy(busy[1]),
      .done(done[1]),
      .sclk(sclk[1]),
      .csb(csb[1]),
      .din(din[1]),
      .dout(dout[1])
      );

   spi_master #(.DATA_WIDTH(DATA_WIDTH),
        .CLK_DIVIDER_WIDTH(CLK_DIVIDER_WIDTH))
     spi_master2
     (.clk(clk),
      .resetb(resetb),
      .CPOL(1'b0),
      .CPHA(1'b1),
      .clk_divider(clk_divider),
      .go(go),
      .datai(datai),
      .datao(master_datao2),
      .busy(busy[2]),
      .done(done[2]),
      .sclk(sclk[2]),
      .csb(csb[2]),
      .din(din[2]),
      .dout(dout[2])
      );
   
   spi_master #(.DATA_WIDTH(DATA_WIDTH),
        .CLK_DIVIDER_WIDTH(CLK_DIVIDER_WIDTH))
     spi_master3
     (.clk(clk),
      .resetb(resetb),
      .CPOL(1'b1),
      .CPHA(1'b1),
      .clk_divider(clk_divider),
      .go(go),
      .datai(datai),
      .datao(master_datao3),
      .busy(busy[3]),
      .done(done[3]),
      .sclk(sclk[3]),
      .csb(csb[3]),
      .din(din[3]),
      .dout(dout[3])
      );
   

   spi_slave #(.DATA_WIDTH(DATA_WIDTH))
     spi_slave0
     (.CPOL(1'b0),
      .CPHA(1'b0),
      .datai(slave_data0),
      .datao(slave_data0),
      .sclk(sclk[0]),
      .csb(csb[0]),
      .din(din[0]),
      .dout(dout[0])
      );

   spi_slave #(.DATA_WIDTH(DATA_WIDTH))
     spi_slave1
     (.CPOL(1'b1),
      .CPHA(1'b0),
      .datai(slave_data1),
      .datao(slave_data1),
      .sclk(sclk[1]),
      .csb(csb[1]),
      .din(din[1]),
      .dout(dout[1])
      );

   spi_slave #(.DATA_WIDTH(DATA_WIDTH))
     spi_slave2
     (.CPOL(1'b0),
      .CPHA(1'b1),
      .datai(slave_data2),
      .datao(slave_data2),
      .sclk(sclk[2]),
      .csb(csb[2]),
      .din(din[2]),
      .dout(dout[2])
      );

   spi_slave #(.DATA_WIDTH(DATA_WIDTH))
     spi_slave3
     (.CPOL(1'b1),
      .CPHA(1'b1),
      .datai(slave_data3),
      .datao(slave_data3),
      .sclk(sclk[3]),
      .csb(csb[3]),
      .din(din[3]),
      .dout(dout[3])
      );

   
   
   initial begin
      clk =  0;
      resetb = 0;
      go = 0;
      datai = 4'h0;
      
      
      $dumpfile("spi_test.vcd");
      $dumpvars(0, tb);
      
    
      #40 resetb = 1;

      #40 $display("Testing spi_master in echo mode");
      send(9);

      repeat(3) @(posedge clk);
      
      send(6);
      test_echo();
      
      
      #40 $finish;
   end
   

   always #1 clk = !clk;


   task test_echo;
      reg passing;
      begin
     passing = 1;
     
     if( master_datao0 !== datai_prev) begin
       $display("FAIL mode=0: datai=0x%x datao=0x%x", datai_prev, master_datao0);
        passing = 0;
     end else begin
       $display("PASS mode=0: datai=0x%x datao=0x%x", datai_prev, master_datao0);
     end
     if( master_datao1 !== datai_prev) begin
       $display("FAIL mode=1: datai=0x%x datao=0x%x", datai_prev, master_datao1);
        passing = 0;
     end else begin
       $display("PASS mode=1: datai=0x%x datao=0x%x", datai_prev, master_datao1);
     end
     if( master_datao2 !== datai_prev) begin
       $display("FAIL mode=2: datai=0x%x datao=0x%x", datai_prev, master_datao2);
        passing = 0;
     end else begin
       $display("PASS mode=2: datai=0x%x datao=0x%x", datai_prev, master_datao2);
     end
     if( master_datao3 !== datai_prev) begin
       $display("FAIL mode=3: datai=0x%x datao=0x%x", datai_prev, master_datao3);
        passing = 0;
     end else begin
       $display("PASS mode=3: datai=0x%x datao=0x%x", datai_prev, master_datao3);
     end
       
      end
   endtask
   
   
   task send;
      input [DATA_WIDTH-1:0] send_data;
      begin
     datai_prev <= datai;
     datai      <= send_data;
     @(posedge clk) go <= 1;
     @(posedge clk) go <= 0;
     @(posedge clk);
     while(|busy)
       @(posedge clk);
     @(posedge clk);
      end
   endtask

   


endmodule
