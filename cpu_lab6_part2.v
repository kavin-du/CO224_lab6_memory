module testbench;
    reg[31:0] INSTRUCTION_t;                // 32'b10010 001 00000000 00000001 00000010;
    wire[31:0] PC_t;                         //     opcode    desti   source1   source2
    reg CLK, RESET;
    reg[31:0] PC_output;
    integer j;

    wire readcache_t;
    wire writecache_t;
    wire busywait_cache_t;
    wire[7:0] cacheReadOutput_t;
    wire[5:0] address_from_cache_t;


    wire readmem_t;
    wire writemem_t;
    wire busywait_mem_t;
    wire[7:0] aluResult_t;
    wire[31:0] read_data_from_mem_t;
    wire[31:0] writedata_to_mem_t;
    wire[7:0] reg1output_t; // input data to write for cache

                                       
    reg[7:0] instr_mem[0:1023]; // 1byte x 1024 word registers

    cpu mycpu(PC_t, INSTRUCTION_t, CLK, RESET, readcache_t, writecache_t, busywait_cache_t, aluResult_t, cacheReadOutput_t, reg1output_t);

    cache mycache(
        .clock(CLK),
        .reset(RESET),
        .read(readcache_t), 
        .write(writecache_t),
        .address(aluResult_t),
        .writedata(reg1output_t),
        .readdata(cacheReadOutput_t),
        .busywait(busywait_cache_t), 

        .mem_read(readmem_t),
        .mem_write(writemem_t),
        .mem_address(address_from_cache_t),
        .mem_writedata(writedata_to_mem_t),
        .mem_readdata(read_data_from_mem_t),
        .mem_busywait(busywait_mem_t)
    );

    data_memory mydata_memory(
        .clock(CLK), 
        .reset(RESET), 
        .read(readmem_t), 
        .write(writemem_t), 
        .address(address_from_cache_t), 
        .writedata(writedata_to_mem_t), 
        .readdata(read_data_from_mem_t), 
        .busywait(busywait_mem_t)
    );

    initial
    begin
        $dumpfile("wavedata.vcd");
        $dumpvars(0, testbench);
        for (j = 0; j < 8; j = j + 1) $dumpvars(0, mycpu.myreg_file.registers[j]); // printing registers 0-7
        for (j = 0; j < 8; j = j + 1) $dumpvars(0, mycache.tag_array[j]); // printing tag array
        for (j = 0; j < 8; j = j + 1) $dumpvars(0, mycache.data_block[j]); // printing data array
        
    end

    always begin
        #4 CLK = ~CLK; // changed clock period to 8 units
    end

    initial begin

        CLK = 1'b1;
        RESET = 1'b0;
        #5
        RESET = 1'b1;
        #6
        RESET = ~RESET;
        
        #1400 $finish; 
    end
    always @(PC_t) begin
        PC_output = PC_t; // copy the updated pc value to new variable just for easyness
    end

    always @(posedge CLK) begin
        #2 // instruction memory read
        INSTRUCTION_t = {instr_mem[32'd0+PC_output], instr_mem[32'd1+PC_output], instr_mem[32'd2+PC_output],instr_mem[32'd3+PC_output]};
    end

    initial begin 
        // copying 32bit instruction to concatenated 1 byte four elements
        {instr_mem[32'd0], instr_mem[32'd1], instr_mem[32'd2],instr_mem[32'd3]} <= 32'b10000000000000010000000001100000; // loadi 1 0x60
        {instr_mem[32'd4], instr_mem[32'd5], instr_mem[32'd6],instr_mem[32'd7]} <= 32'b01111000000000000000000100101000; // swi 1 0x28      -- write hit
        {instr_mem[32'd8], instr_mem[32'd9], instr_mem[32'd10],instr_mem[32'd11]} <= 32'b10000000000000100000000001100001; // loadi 2 0x61  
        {instr_mem[32'd12], instr_mem[32'd13], instr_mem[32'd14],instr_mem[32'd15]} <= 32'b01111000000000000000001010001000; // swi 2 0x88  -- diff tag -- write miss
        {instr_mem[32'd16], instr_mem[32'd17], instr_mem[32'd18],instr_mem[32'd19]} <=32'b11101000000001000000000010001000; // lwi 4 0x88 -- read hit
        {instr_mem[32'd20], instr_mem[32'd21], instr_mem[32'd22],instr_mem[32'd23]} <=32'b11101000000000110000000000101000; // lwi 3 0x28  -- read miss and dirty
        {instr_mem[32'd24], instr_mem[32'd25], instr_mem[32'd26],instr_mem[32'd27]} <=32'b11101000000001010000000010001000; // lwi 5 0x88     read miss not dirty
        {instr_mem[32'd28], instr_mem[32'd29], instr_mem[32'd30],instr_mem[32'd31]} <=32'b01111000000000000000000101111110; // swi 1 0x7E     write hit
        {instr_mem[32'd32], instr_mem[32'd33], instr_mem[32'd34],instr_mem[32'd35]} <=32'b01111000000000000000001001111101; // swi 2 0x7D    write miss, tag match, dirty
        {instr_mem[32'd36], instr_mem[32'd37], instr_mem[32'd38],instr_mem[32'd39]} <= 32'b01111000000000000000000101011101; // swi 1 0x5D    miss
        {instr_mem[32'd40], instr_mem[32'd41], instr_mem[32'd42],instr_mem[32'd43]} <= 32'b10001000000000000000000000000001; // mov 0 1
        {instr_mem[32'd44], instr_mem[32'd45], instr_mem[32'd46],instr_mem[32'd47]} <= 32'b10000000000000110000000001011101; // loadi 3 0x5D 
        {instr_mem[32'd48], instr_mem[32'd49], instr_mem[32'd50],instr_mem[32'd51]} <= 32'b11100000000001000000000000000011; // lwd 4 3
        {instr_mem[32'd52], instr_mem[32'd53], instr_mem[32'd54],instr_mem[32'd55]} <= 32'b01110000000000000000000000000010; // swd 0 2


       
    end
endmodule



/*********************
    my opcode definitions--
    1st bit of opcode decides whether its writing to register or not.
    next 4 bits are unique to each operation, it will control the desired second operand.
    final 3 bits are the aluop

    loadi  = "10000 000";
	mov    = "10001 000";
	add    = "10010 001";
	sub    = "10011 001";
	and    = "10100 010";
	or 	   = "10100 011";
	j	   = "00001 100";
	beq	   = "00000 101";
    bne  = "00011 101"; branch not equal
    sll  = "11000 110"; left logical shift
    srl  = "11001 111"; right logical shift

    // added for lab6_part1
    lwd 	= "11100 000";
	lwi 	= "11101 000";
	swd 	= "01110 000";
	swi 	= "01111 000";


    
*******************/

// no additional control unit has implemented, all funtionalities are in the cpu module
module cpu(PC, INSTRUCTION, CLK, RESET, READMEM, WRITEMEM, BUSYWAIT, aluResult, memReadOutput, reg1output);  
    
    input[31:0] INSTRUCTION;  // 32 bit instruction
    output reg[31:0] PC; // 32 bit program counter
    input CLK, RESET;
    
    
    reg write;  // write enable signal
    reg[7:0] destination;
    reg[7:0] opcode; 
    reg[2:0] aluop;
    reg[7:0] source1;
    reg[7:0] source2;

    reg[7:0] immediate; // immediate value
    reg[7:0] alusrc1;  
    reg[7:0] registerIn;  // register input for writing data to register
    
    // these wires can be declared as "output"
    wire[7:0] aluResult_wire; // result of alu operation/module
    output reg[7:0] aluResult;
    wire[7:0] reg1output_wire; // getting the output from register file
    output reg[7:0] reg1output; // sending reg1output to memory module
    wire[7:0] reg2output;
    wire[7:0] twoscomplement; // twoscomplement second operand
    wire[31:0] pcUpdaterout;  // updated program counter value from dedicated adder
    wire[7:0] alusrc2; // this is generated by the source 2 mux(this is the output of that mux--immediate or register value)
    wire zerooutput; // zero result output for beq 

    input[7:0] memReadOutput;  // readed value from memory/ cache
    wire[7:0] registerInput; // either aluresult or memory read
    input BUSYWAIT;
    output reg WRITEMEM;  // memory write enable signal
    output reg READMEM;    // memory read enable signal

    wire[31:0] branchResult;

    always @(INSTRUCTION) begin
        destination <= INSTRUCTION[23:16];  // either the register to be written to in the register file,or an immediate value (jump or branch target offset).
        source1 <= INSTRUCTION[15:8]; // 1 st operand to be read from the register file.    
        source2 <= INSTRUCTION[7:0];  // 2 nd operand from the register file, or an immediate value (loadi).
        immediate <= INSTRUCTION[7:0]; // copy as immediate value    
        #1 // control signal generate
        write <= INSTRUCTION[31]; // first bit of my opcode is write enable signal
        opcode <= INSTRUCTION[31:24];
        aluop <= INSTRUCTION[26:24]; // aluop is the last 3 bits of my opcode -- opcode[2:0]
        
    end
    always @(BUSYWAIT) begin        
        write =(~BUSYWAIT & INSTRUCTION[31]);  // if busywait enabled, stop writing to register file
    end
    
    // instantiating submodules
    reg_file myreg_file(.IN(registerIn), .OUT1(reg1output_wire), .OUT2(reg2output), .INADDRESS(destination[2:0]), 
        .OUT1ADDRESS(source1[2:0]), .OUT2ADDRESS(source2[2:0]), .WRITE(write), .CLK(CLK), .RESET(RESET));

    alu myalu(.DATA1(alusrc1), .DATA2(alusrc2), .RESULT(aluResult_wire), .SELECT(aluop), .ZERO(zerooutput));

    twosComplement mytwo(.NUMBER(reg2output), .OUT(twoscomplement));

    pcUpdater mypcUpdater(.pc(PC), .clk(CLK), .reset(RESET), .pcout(pcUpdaterout));

    // this mux is selecting the source2 or immediate value depending on the opcode
    src2mux my_src2mux(.ALUOP(aluop), .OPCODE(opcode), .ALUSRC2(alusrc2), .IMMEDIATE(immediate), .TWOSCOMPLEMENT(twoscomplement), .REG2OUTPUT(reg2output));

    branchValue mybranchValue(.PC(PC), .OFFSET(destination), .OUT(branchResult));
    
    
    inputForRegisterIn my_inputForRegisterIn(.INSTRUCTION(INSTRUCTION), .OPCODE(opcode), .ALURESULT(aluResult_wire), .MEMREADOUTPUT(memReadOutput), .OUTPUTVALUE(registerInput));

    always@(INSTRUCTION) begin
        #1    // control signals are generating after 1 time unit
        READMEM = 1'b0; // resetting readmem, writemem at every new instruction
        WRITEMEM = 1'b0;
        // comparing opcodes
        if((INSTRUCTION[31:24] == 8'b11100000) || (INSTRUCTION[31:24] == 8'b11101000)) begin  // lwd, lwi 
            #2  // data memory access 2 units delay
            READMEM = 1'b1;
            WRITEMEM = 1'b0;
        end
        else if((INSTRUCTION[31:24] == 8'b01110000) || (INSTRUCTION[31:24] == 8'b01111000)) begin // swd, swi
            #2  // data memory access 2 units delay
            WRITEMEM = 1'b1;
            READMEM = 1'b0;
        end
        else begin
            WRITEMEM = 1'b0;
            READMEM = 1'b0;
        end
    end
    always@(*) begin
        aluResult <= aluResult_wire; // output alu result value from cpu to mem module
    end
    always@(*) begin
        alusrc1 <= reg1output_wire; // always providing the register output to alu source 1
        reg1output <= reg1output_wire; // also providing as output from cpu to memory module
    end
    always @(registerInput)
        registerIn <= registerInput;

    always @(posedge CLK) begin
        #1
    	if(!RESET && !BUSYWAIT) begin
            if (opcode == 8'b00001100) PC = branchResult; // new branch(pc value) for jump, if it is jump instruction no need to check the zero output
            else if(opcode == 8'b00000101) begin // branch value for beq
                if(zerooutput == 1'b1) PC = branchResult; 
                else PC = pcUpdaterout;
            end
            else if(opcode == 8'b00011101) begin // branch value for bne
                if(zerooutput == 1'b0) PC = branchResult; 
                else PC = pcUpdaterout;
            end
            else PC = pcUpdaterout;   // every other instruction is not jump or beq
        end    
    end

    always @(RESET) begin
    	if(RESET)
        	#1 PC = pcUpdaterout;  // pc update(write to pc)       
    end

endmodule

// this mux decides which value is passed to register write port
// either alu result or readed value from memory
module inputForRegisterIn(INSTRUCTION, OPCODE, ALURESULT, MEMREADOUTPUT, OUTPUTVALUE);
    input[31:0] INSTRUCTION;
    input[7:0] OPCODE;
    input[7:0] ALURESULT;
    input[7:0] MEMREADOUTPUT;
    output reg[7:0] OUTPUTVALUE;

    always @(INSTRUCTION, MEMREADOUTPUT, ALURESULT) begin
        if((OPCODE[6:3] == 4'b1100) || (OPCODE[6:3] == 4'b1101)) begin // those 4 bits are unique to lwd, lwi
            OUTPUTVALUE = MEMREADOUTPUT;                               // so when lwd, lwi memory readed value will be passed
        end
        else begin
            OUTPUTVALUE = ALURESULT; 
        end
    end



endmodule

module cache(
    clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
	busywait, 

    mem_read,
    mem_write,
    mem_address,
    mem_writedata,
    mem_readdata,
    mem_busywait
);

    input clock;
    input reset;
    input read;
    input write;
    input[7:0] address;
    input[7:0] writedata;
    output reg[7:0] readdata;
    output reg busywait;

    output reg mem_read;
    output reg mem_write;
    output reg[5:0] mem_address;   // 6 bit address for main memory access
    output reg[31:0] mem_writedata;  // data block for write to main mem
    input[31:0] mem_readdata;       // block read from main mem
    input mem_busywait;

    reg[7:0] valid;
    reg[7:0] dirty;
    reg[2:0] tag_array [7:0];  // array with 3 bit tag
    reg[31:0] data_block[7:0];   // 4 byte data blocks

    reg[2:0] tag;
    reg[2:0] index;
    reg[1:0] offset;

    reg readhit;  // read hit signal
    reg writehit; // write hit signal -- two seperate hit signals used to be more explicit

    always @(address) // seperate address into tag, index, offset
    begin
        tag = address[7:5];
        index = address[4:2];
        offset = address[1:0];
    end


    //Detecting an incoming cache memory access
    reg readaccess, writeaccess;
    always @(read, write)
    begin
        busywait = (read || write)? 1 : 0;
        readaccess = (read && !write)? 1 : 0;
        writeaccess = (!read && write)? 1 : 0;
    end

    always @(*) // read hit determine
    begin
        if(readaccess) 
        begin
            #1
            if(valid[index] && tag_array[index] == tag)  // comparing tag and valid bit
                readhit = 1;
            else
                readhit = 0;            
        end
        else  // if the instruction is not a cache read, then readhit is de-asserted
            readhit = 0;
    end

    always @(*)  // write hit determine
    begin
        if(writeaccess)
        begin
            #1
            if(!valid[index]) // very first write, not need to compare other things
                writehit = 1;
            else if(tag_array[index] == tag && dirty[index] == 0)  // comparing tag and dirty bits
                writehit = 1;
            else
                writehit = 0;
        end
        else  // if the instruction is not a cache write, then write hit is de-asserted
            writehit = 0;
    end


    always @(readhit, readaccess)  
    begin
        #1
        if(readaccess) 
        begin        
            if(readhit) // read hit
            begin    
                case(offset)      // extracting the correct block using offset   
                    2'b00 : readdata = data_block[index][7:0];
                    2'b01 : readdata = data_block[index][15:8];
                    2'b10 : readdata = data_block[index][23:16];
                    2'b11 : readdata = data_block[index][31:24];
                endcase
                $display("read: %b", data_block[index]);
                // busywait = 0;
            end
        end
        
    end

    always @(writehit)
    begin
        if(writeaccess) 
        begin
            #1
            if(writehit)
            begin
                valid[index] = 1;
                dirty[index] = 1;
                tag_array[index] = tag; 

                case(offset) // write to correct block using offset
                    2'b00 : data_block[index][7:0] = writedata;
                    2'b01 : data_block[index][15:8] = writedata;
                    2'b10 : data_block[index][23:16] = writedata;
                    2'b11 : data_block[index][31:24] = writedata;
                endcase
                $display("write: %b", data_block[index]);
                // busywait = 0; 
            end           
        end
    end

    // when a hit occured resetting busywait at next positive edge

    always @(posedge clock)
    begin
        if((readhit || writehit) && !mem_busywait)
        // if(readhit || writehit)
            busywait = 0;
    end


    /* Cache controller FSM */

    parameter IDLE_STATE = 3'b000, MEM_READ_STATE = 3'b001, MEM_WRITE_STATE = 3'b010,
                                   WRITE_TO_CACHE_ON_MISS = 3'b011;
    reg[2:0] state, next_state;


    always @(*)
    begin
        case (state) 
            IDLE_STATE: 
                if(busywait && readaccess && !writeaccess && valid[index] && !readhit && !dirty[index]) begin // read miss but not dirty-- read from memory 
                    next_state = MEM_READ_STATE;
                    $display("got 1"); // done
                end
                else if(busywait && readaccess && !writeaccess && valid[index] && !readhit && dirty[index]) begin // read miss and dirty -- write old block to memory
                    next_state = MEM_WRITE_STATE;
                    $display("got 2");  // done
                end
                else if(busywait && writeaccess && !readaccess && !writehit && tag_array[index]==tag && dirty[index]) begin // write miss, tag match but dirty -- write old block to memory
                    next_state = MEM_WRITE_STATE;
                    $display("got 3"); // done
                end
                else if(busywait && writeaccess && !readaccess && !writehit && tag_array[index] != tag && !dirty[index]) begin // write miss, tag mismatch and not dirty -- read new block from memory
                    next_state = MEM_READ_STATE;
                    $display("got 4");
                end
                else if(busywait && writeaccess && !readaccess && !writehit && tag_array[index] != tag && dirty[index]) begin  // tag not match and dirty  -- write old block to memory
                    next_state = MEM_WRITE_STATE;
                    $display("got 5");          // done
                end

                else begin
                    next_state = IDLE_STATE;
                    $display("got 6",$time); // done
                end


            MEM_READ_STATE:  // if memory read is done always write to cache(newly readed block)
                if(readaccess) 
                    next_state = WRITE_TO_CACHE_ON_MISS;
                else if(writeaccess)
                    next_state = WRITE_TO_CACHE_ON_MISS;

            MEM_WRITE_STATE:              
                if(readaccess && !writeaccess && valid[index] && !readhit && !dirty[index]) // read miss and dirty 2nd step 
                    next_state = MEM_READ_STATE; // after writing old block, now fetch the new block from memory
                else if(writeaccess && !readaccess && !writehit && tag_array[index] == tag && !dirty[index])  // write miss, tag match but dirty 2nd step
                begin   
                    next_state = IDLE_STATE;  // after writing old block to memory, the dirty bit will de assert, then by asynchronus logic, new data will written(writing was sensitive to dirty bit), so it is not need to handle by a seperate state
                    $display("x is here");
                end
                else if(writeaccess && !readaccess && !writehit && tag_array[index] != tag && !dirty[index]) // write miss -tag not match and dirty 2nd step
                    next_state = MEM_READ_STATE;  // after writing the old block, fetch new block from memory

            WRITE_TO_CACHE_ON_MISS:
                next_state = IDLE_STATE;  // after each memory read, fetched data need to write to cache, then go to idle state
 

        endcase


    end



    always @(*) 
    begin
        case (state)
            IDLE_STATE: 
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 8'dx;
                mem_writedata = 8'dx;
                busywait = 0;
                $display("--------IDLE_STATE");
            end

            MEM_READ_STATE:
            begin
                mem_read = 1;
                mem_write =  0;
                mem_address = {tag, index};  // reading new block
                mem_writedata = 32'dx;
                busywait = 1;
                // dirty[index] = 0;
                $display("--------MEM READ STATE, %d", $time);
            end

            MEM_WRITE_STATE:    
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {tag_array[index], index};   // old tag should be write
                mem_writedata = data_block[index]; 
                dirty[index] = 0;
                busywait = 1;
                $display("mem write data: %b, %d", data_block[index], $time);
                $display("--------MEM_WRITE_STATE");
            end

            WRITE_TO_CACHE_ON_MISS:
            begin
                #1
                data_block[index] = mem_readdata;
                dirty[index] = 0;
                tag_array[index] = tag; // updating tag
                busywait = 1;
                $display("------WRITE_TO_CACHE_ON_MISS");
                $display("write on miss: %b, %d", mem_readdata, $time);
            end
            

        endcase
    end

    // state transitioning
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE_STATE;
        else if(!mem_busywait)   // if memory operation is pending, do not change the state
            state = next_state;
    end


    integer i;

    //Reset cache
    always @(posedge reset)
    begin
        if (reset) 
        begin
            for (i=0;i<8; i=i+1)
                valid[i] = 0;
            
            busywait = 0;
            readaccess = 0;
            writeaccess = 0;
        end
    end

endmodule

// provided data memory module 
module data_memory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
	busywait
);
input				clock;
input           	reset;
input           	read;
input           	write;
input[5:0]      	address;
input[31:0]     	writedata;
output reg [31:0]	readdata;
output reg      	busywait;

//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
	if(readaccess)
	begin
        readdata[7:0]   = #40 memory_array[{address,2'b00}];
		readdata[15:8]  = #40 memory_array[{address,2'b01}];
		readdata[23:16] = #40 memory_array[{address,2'b10}];
		readdata[31:24] = #40 memory_array[{address,2'b11}];
		busywait = 0;
		readaccess = 0;
	end
	if(writeaccess)
	begin
        memory_array[{address,2'b00}] = #40 writedata[7:0];
		memory_array[{address,2'b01}] = #40 writedata[15:8];
		memory_array[{address,2'b10}] = #40 writedata[23:16];
		memory_array[{address,2'b11}] = #40 writedata[31:24];
		busywait = 0;
		writeaccess = 0;
	end
end

integer i;

//Reset memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end

endmodule


// module for calculating branch value for jump, beq 
module branchValue(PC, OFFSET, OUT);
    input[31:0] PC;
    input[7:0] OFFSET; // offset provided as 8 bits
    output reg[31:0] OUT; 

    always @(PC, OFFSET)
        #2  // two times delay        
        OUT = (PC+ 32'd4) + {{24{OFFSET[7]}} , (OFFSET<<2)};  // left shift two times and concatenate to the sign extended MSB
        


endmodule


// module for deciding the alusrc2 is either immediate value or register output
module src2mux(ALUOP, OPCODE, ALUSRC2, IMMEDIATE, TWOSCOMPLEMENT, REG2OUTPUT);
    input[2:0] ALUOP;
    input[7:0] OPCODE;   
    input[7:0] IMMEDIATE;
    input[7:0] TWOSCOMPLEMENT;
    input[7:0] REG2OUTPUT;

    output reg[7:0] ALUSRC2;

    // these things are decided by 3-6 bits of my opcode, those are unique to each operation
    always @(*) begin
        case(ALUOP)
            // 3'b000 : ALUSRC2 <= OPCODE[6:3] == 4'b0000 ? IMMEDIATE : REG2OUTPUT;    // loadi, mov
            3'b000 : begin
                        if((OPCODE[6:3] == 4'b0000) || (OPCODE[6:3] == 4'b1101) || (OPCODE[6:3] == 4'b1111)) begin // loadi, lwi, swi
                            ALUSRC2 <= IMMEDIATE;
                        end
                        else begin   // mov, lwd, swd
                            ALUSRC2 <= REG2OUTPUT;
                        end
                    end


            3'b001 : ALUSRC2 <= OPCODE[6:3] == 4'b0011 ? TWOSCOMPLEMENT : REG2OUTPUT; // add, sub
            3'b010 : ALUSRC2 <= REG2OUTPUT; // and
            3'b011 : ALUSRC2 <= REG2OUTPUT; // or

            3'b101 : ALUSRC2 <= TWOSCOMPLEMENT; // beq, bne
            3'b110 : ALUSRC2 <= IMMEDIATE; // sll - logical left shift
            3'b111 : ALUSRC2 <= IMMEDIATE; // srl - logical right shift
        endcase
    end

endmodule

// dedicated adder for pc update.
module pcUpdater(pc, clk, reset, pcout);
    input[31:0] pc;
    input reset;
    input clk;
    output reg[31:0] pcout;

    always @(pc) begin
        #1 pcout = pc + 32'd4; // pc update delay changed to #1 according to lab6
    end

    always @(reset) begin
        if(reset)        
            pcout = -32'd4; // at the reset, writing -4
    end
endmodule

// two's complement module for substract operation
module twosComplement(NUMBER, OUT);
    input[7:0] NUMBER;
    output reg[7:0] OUT;
    integer i; 
    
    // outputting the two's complement of second operand
    always @(NUMBER) begin
        #1 // 1 time unit delay -- preparation lab6
        for(i=0; i<8; i=i+1) begin
            OUT[i] = NUMBER[i] == 1 ? 0 : 1;
        end
        OUT = OUT + 8'd1;
    end
endmodule





///// below modules are from the previous submissions






///////////// modified ALU ---- add two time units
module alu(DATA1, DATA2, RESULT, SELECT, ZERO);

    input[7:0] DATA1, DATA2; // declaring 8bit input ports
    input[2:0] SELECT;  // 3bit input port for ALUOp code
    output reg[7:0] RESULT; // 8bit output
    output reg ZERO;
    integer i, a;
    
    always @(DATA2) begin
        a = DATA2;  // making shifting amount as a integer to make operations in for loop
    end


    
    always @ (DATA1 or DATA2 or SELECT)begin  // always block execute if either of inputs are changed
        case (SELECT)
            3'b001 : #2 begin RESULT = DATA1 + DATA2; ZERO = 1'b0; end // add-sub operation aluopcode 001                              
            3'b000 : #1 begin RESULT = DATA2; ZERO = 1'b0; end         // mov-loadi aluopcode 000, moving data2 to result
            3'b010 : #1 begin RESULT = DATA1 & DATA2; ZERO = 1'b0; end  // bitwise and aluopcode 010
            3'b011 : #1 begin RESULT = DATA1 | DATA2; ZERO = 1'b0; end  // bitwise or aluopcode 011
            3'b101 : #2 begin RESULT = DATA1 + DATA2; // beq aluopcode 101
                              if(~(RESULT[0]|RESULT[1]|RESULT[2]|RESULT[3]|RESULT[4]|RESULT[5]|RESULT[6]|RESULT[7])) ZERO = 1'b1; // checking if the result is zero
                              else ZERO = 1'b0;                              
                        end  
            3'b110 : #1 begin // sll - logical shift left
                                if(a > 7) begin // if shifting amount is greater than 7, answer should be 0
                                    RESULT = 8'd0; 
                                end
                                else begin
                                    for(i=0; i<8-a; i=i+1) begin
                                        RESULT[i+a] = DATA1[i];
                                    end
                                    for(i=0; i<a; i=i+1) begin // making LSB as 0s
                                        RESULT[i] = 1'b0;
                                    end
                                end
                                ZERO = 1'b0; 
                        end 
            3'b111 : #1 begin // srl - logical shift right
                                if(a > 7) begin // if shifting amount is greater than 7, answer should be 0
                                    RESULT = 8'd0; 
                                end
                                else begin
                                    for(i=7; i>=a; i=i-1) begin
                                        RESULT[i-a] = DATA1[i];
                                    end
                                    for(i=7; i>7-a; i=i-1) begin // filling MSB as 0s
                                        RESULT[i] = 1'b0;
                                    end

                                end
                                ZERO = 1'b0; 
                        end 
            default : ZERO = 1'b0;  // since aluopcode for jump is not used in here, the zero output is disabled by default
        endcase        
    end

endmodule

/////////////////////////// REGISTER FILE \\\\\\\\\\\\\\\\\\\\\\\\\\\\
module reg_file (IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);

    input[7:0] IN; // input data
    output reg [7:0] OUT1, OUT2; // data output
    input[2:0] INADDRESS, OUT1ADDRESS, OUT2ADDRESS; // 3bit addressess corresponding to register numbers
    input WRITE, CLK, RESET;

    output reg[7:0] registers[0:7]; // 8bit eight registers

    integer l; // for the loop, l is for the loop for clearing registers(if reset enabled)

    // since reading happening always, when either one of the register's content is change, it should show the output
    // 2 times units after change happens(after resetting and writing)
    always @(registers[0] or registers[1] or registers[2] or registers[3] or registers[4] or registers[5] or registers[6] or registers[7]) begin
        
        #2
        OUT1 <= registers[OUT1ADDRESS];
        OUT2 <= registers[OUT2ADDRESS];
        // $monitor($time, " reg[%d]: %d reg[%d]: %d", OUT1ADDRESS, OUT1, OUT2ADDRESS, OUT2);
        
    end

    // except the content changing of the register, if either one of read address changes it should give the
    // corresponding output.
    // but if the reset is happening we should give priority to the reset. 
    // since it will be already displaying the output after reset(resetting is like writing 0s)
    always @(OUT1ADDRESS or OUT2ADDRESS) begin 
        if(!RESET) begin
            #2
            OUT1 <= registers[OUT1ADDRESS];
            OUT2 <= registers[OUT2ADDRESS];
            // $monitor($time, " reg[%d]: %d reg[%d]: %d", OUT1ADDRESS, OUT1, OUT2ADDRESS, OUT2);
        end          
    end

    // at each pos edge of the clock writing to the register after 2 time units
    always @(posedge CLK) begin  
        #1  
        if(WRITE) begin
        //    #1 // changed -- preparation lab 6
            registers[INADDRESS] = IN;     
            // $monitor($time, " reg[%d]: %d", INADDRESS, IN);                   
        end        
    end

    // if the reset enabled, making all registers to zeroes
    always @ (RESET) begin   
        if(RESET) begin
            #1  // changed -- preparation lab 6
            for(l=0; l<8; l=l+1) begin
                registers[l] <= 8'd0;
            end
        end
    end
endmodule



