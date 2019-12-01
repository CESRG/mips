
----------------------------------------------------------------------------------------------


library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity testbench is
end;

architecture test of testbench is
  component top
    port(clk, reset:           in  STD_LOGIC;
         writedata, dataadr:   out STD_LOGIC_VECTOR(31 downto 0);
         memwrite:             out STD_LOGIC);
  end component;
  signal writedata, dataadr:    STD_LOGIC_VECTOR(31 downto 0);
  signal clk, reset,  memwrite: STD_LOGIC;
begin

  -- instantiate device to be tested
  dut: top port map(clk, reset, writedata, dataadr, memwrite);

  -- Generate clock with 10 ns period
  process begin
    clk <= '1';
    wait for 5 ns;
    clk <= '0';
    wait for 5 ns;
  end process;

  -- Generate reset for first two clock cycles
  process begin
    reset <= '1';
    wait for 22 ns;
    reset <= '0';
    wait;
  end process;

  -- check that 7 gets written to address 84 at end of program
  process (clk) begin
    if (clk'event and clk = '0' and memwrite = '1') then
      if (to_integer(dataadr) = 84 and to_integer(writedata) = 7) then
        report "NO ERRORS: Simulation succeeded" severity failure;
      elsif (dataadr /= 80) then
        report "Simulation failed" severity failure;
      end if;
    end if;
  end process;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity top is -- top-level design for testing
  port(clk, reset:           in     STD_LOGIC;
       writedata, dataadr:   buffer STD_LOGIC_VECTOR(31 downto 0);
       memwrite:             buffer STD_LOGIC);
end;

architecture test of top is
  component mips 
    port(clk, reset:        in  STD_LOGIC;
         pc:                out STD_LOGIC_VECTOR(31 downto 0);
         instr:             in  STD_LOGIC_VECTOR(31 downto 0);
         memwrite:          out STD_LOGIC;
         aluout, writedata: out STD_LOGIC_VECTOR(31 downto 0);
         readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component imem
    port(a:  in  STD_LOGIC_VECTOR(5 downto 0);
         rd: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component dmem
    port(clk, we:  in STD_LOGIC;
         a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
         rd:       out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  signal pc, instr, 
         readdata: STD_LOGIC_VECTOR(31 downto 0);
begin
  -- instantiate processor and memories
  mips1: mips port map(clk, reset, pc, instr, memwrite, dataadr, 
                       writedata, readdata);
  imem1: imem port map(pc(7 downto 2), instr);
  dmem1: dmem port map(clk, memwrite, dataadr, writedata, readdata);
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity dmem is -- data memory
  port(clk, we:  in STD_LOGIC;
       a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
       rd:       out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of dmem is
begin
  process is
    type ramtype is array (63 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
    variable mem: ramtype;
  begin
    -- read or write memory
    loop
      if clk'event and clk = '1' then
          if (we = '1') then mem(to_integer(a(7 downto 2))) := wd;
          end if;
      end if;
      rd <= mem(to_integer(a(7 downto 2))); 
      wait on clk, a;
    end loop;

  end process;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity imem is -- instruction memory
  port(a:  in  STD_LOGIC_VECTOR(5 downto 0);
       rd: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of imem is
begin
  process is
    file mem_file: TEXT;
    variable L: line;
    variable ch: character;
    variable i, index, result: integer;
    type ramtype is array (63 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
    variable mem: ramtype;
  begin
    -- initialize memory from file
    for i in 0 to 63 loop -- set all contents low
      mem(i) := (others => '0'); 
    end loop;
    index := 0; 
    FILE_OPEN(mem_file, "memfile.dat", READ_MODE);
    while not endfile(mem_file) loop
      readline(mem_file, L);
      result := 0;  
      for i in 1 to 8 loop
        read(L, ch);
        if '0' <= ch and ch <= '9' then 
            result := character'pos(ch) - character'pos('0');
        elsif 'a' <= ch and ch <= 'f' then
           result := character'pos(ch) - character'pos('a')+10;
        else report "Format error on line " & integer'image(index)
             severity error;
        end if;
        mem(index)(35-i*4 downto 32-i*4) :=to_std_logic_vector(result,4);
      end loop;
      index := index + 1;
    end loop;

    -- read memory
    loop
      rd <= mem(to_integer(a));
      wait on a;
    end loop;
  end process;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity mips is -- single cycle MIPS processor
  port(clk, reset:        in  STD_LOGIC;
       pc:                out STD_LOGIC_VECTOR(31 downto 0);
       instr:             in  STD_LOGIC_VECTOR(31 downto 0);
       memwrite:          out STD_LOGIC;
       aluout, writedata: out STD_LOGIC_VECTOR(31 downto 0);
       readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
end;

architecture struct of mips is
  component controller
    port(op, funct:          in  STD_LOGIC_VECTOR(5 downto 0);
         memtoreg, memwrite: out STD_LOGIC;
         branch, alusrc, c:   out STD_LOGIC;
         regdst, regwrite:   out STD_LOGIC;
         jump:               out STD_LOGIC;
         alucontrol:         out STD_LOGIC_VECTOR(2 downto 0));
  end component;   
  
  component datapath
    port(clk, reset, c:     in  STD_LOGIC;
         memtoreg, branch:   in  STD_LOGIC;
         alusrc, regdst:    in  STD_LOGIC;
         regwrite, jump:    in  STD_LOGIC;
     memwrite:      in std_logic;  
     memwritepip:   out std_logic;
         alucontrol:        in  STD_LOGIC_VECTOR(2 downto 0);
         pc:                buffer STD_LOGIC_VECTOR(31 downto 0);
         instr:             in STD_LOGIC_VECTOR(31 downto 0);
         aluout, writedata: buffer STD_LOGIC_VECTOR(31 downto 0);
         readdata:          in  STD_LOGIC_VECTOR(31 downto 0);
     op, funct:         out std_logic_vector(5 downto 0));
  end component;
  
  signal memtoreg, alusrc, regdst, regwrite, jump, branch, s_c, s_memwrite: STD_LOGIC;
  signal alucontrol: STD_LOGIC_VECTOR(2 downto 0);
  signal s_op, s_funct: std_logic_vector(5 downto 0);
begin
  cont: controller port map(s_op, s_funct,
                memtoreg, s_memwrite, branch, alusrc, s_c,
                            regdst, regwrite, jump, alucontrol);
  dp: datapath port map(clk, reset, s_c, memtoreg, branch, alusrc, regdst,
                        regwrite, jump, s_memwrite, memwrite, alucontrol, pc, instr,
                        aluout, writedata, readdata, s_op, s_funct);
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity controller is -- single cycle control decoder
  port(op, funct:          in  STD_LOGIC_VECTOR(5 downto 0);
       memtoreg, memwrite: out STD_LOGIC;
       branch, alusrc, c:   out STD_LOGIC;
       regdst, regwrite:   out STD_LOGIC;
       jump:               out STD_LOGIC;
       alucontrol:         out STD_LOGIC_VECTOR(2 downto 0));
end;

architecture struct of controller is
  component maindec
    port(op:                 in  STD_LOGIC_VECTOR(5 downto 0);
         memtoreg, memwrite: out STD_LOGIC;
         branch, alusrc, c:  out STD_LOGIC;
         regdst, regwrite:   out STD_LOGIC;
         jump:               out STD_LOGIC;
         aluop:              out STD_LOGIC_VECTOR(1 downto 0));
  end component;  
  
  component aludec
    port(funct:      in  STD_LOGIC_VECTOR(5 downto 0);
         aluop:      in  STD_LOGIC_VECTOR(1 downto 0);
         alucontrol: out STD_LOGIC_VECTOR(2 downto 0));
  end component;
  signal aluop:  STD_LOGIC_VECTOR(1 downto 0);
begin
  md: maindec port map(op, memtoreg, memwrite, branch,
                       alusrc, c, regdst, regwrite, jump, aluop);
  ad: aludec port map(funct, aluop, alucontrol);
end;
----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity aludec is -- ALU control decoder
  port(funct:      in  STD_LOGIC_VECTOR(5 downto 0);
       aluop:      in  STD_LOGIC_VECTOR(1 downto 0);
       alucontrol: out STD_LOGIC_VECTOR(2 downto 0));
end;

architecture behave of aludec is
begin
  process(all) begin
    case aluop is
    when "00" => alucontrol <= "010"; -- add (for lw/sw/addi)
    when "01" => alucontrol <= "110"; -- sub (for beq/bne)
    when "11" => alucontrol <= "001"; -- or  (for ori)
    when others => case funct is      -- R-type instructions
                         when "100000" => alucontrol <= "010"; -- add 
                         when "100010" => alucontrol <= "110"; -- sub
                         when "100100" => alucontrol <= "000"; -- and
                         when "100101" => alucontrol <= "001"; -- or
                         when "101010" => alucontrol <= "111"; -- slt
                         when others   => alucontrol <= "---"; -- ???
                     end case;
    end case;
  end process;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity maindec is -- main control decoder
  port(op:                 in  STD_LOGIC_VECTOR(5 downto 0);
       memtoreg, memwrite: out STD_LOGIC;
       branch, alusrc, c:  out STD_LOGIC;
       regdst, regwrite:   out STD_LOGIC;
       jump:               out STD_LOGIC;
       aluop:              out STD_LOGIC_VECTOR(1 downto 0));
end;

architecture behave of maindec is
  signal controls: STD_LOGIC_VECTOR(9 downto 0);
begin
  process(all) begin
    case op is
      when "000000" => controls <= "1100000100"; -- RTYPE
      when "100011" => controls <= "1010010000"; -- LW
      when "101011" => controls <= "0010100000"; -- SW
      when "000100" => controls <= "0001000010"; -- BEQ
    when "000101" => controls <= "0001000010"; -- BNE   
      when "001000" => controls <= "1010000000"; -- ADDI
      when "000010" => controls <= "0000001000"; -- J 
    when "001101" => controls <= "1010000111";  -- ORI
      when others   => controls <= "----------"; -- illegal op
    end case;
  end process;

  (regwrite, regdst, alusrc, branch, memwrite,
   memtoreg, jump, aluop(1 downto 0), c) <= controls;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity datapath is  -- MIPS datapath
  port( clk, reset, c:     in  STD_LOGIC;
      memtoreg, branch:  in  STD_LOGIC;
      alusrc, regdst:    in  STD_LOGIC;
      regwrite, jump:    in  STD_LOGIC;
      memwrite:      in  std_logic;
      memwritepip:     out std_logic;
      alucontrol:        in  STD_LOGIC_VECTOR(2 downto 0);
      pc:                buffer STD_LOGIC_VECTOR(31 downto 0);
      instr:             in  STD_LOGIC_VECTOR(31 downto 0);
      aluout, writedata: buffer STD_LOGIC_VECTOR(31 downto 0);
      readdata:          in  STD_LOGIC_VECTOR(31 downto 0);
      op, funct:         out std_logic_vector(5 downto 0));
      
end;

architecture struct of datapath is
  component alu
    port( a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
        alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
        result:     buffer STD_LOGIC_VECTOR(31 downto 0);
        zero:       out STD_LOGIC);
  end component;  
  
  component regfile
    port( clk:           in  STD_LOGIC;
        we3:           in  STD_LOGIC;
        ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0);
        wd3:           in  STD_LOGIC_VECTOR(31 downto 0);
        rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
  end component;  
  
  component adder
    port( a, b: in  STD_LOGIC_VECTOR(31 downto 0);
        y:    out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  
  component sl2
    port( a: in  STD_LOGIC_VECTOR(31 downto 0);
        y: out STD_LOGIC_VECTOR(31 downto 0));
  end component; 
  
  component signext                          
    port( a: in  STD_LOGIC_VECTOR(15 downto 0);
          c: in  STD_LOGIC; -- c = '0' arit, c = '1' logical
          y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;  
     
  component mux2 
    generic(width: integer);
    port( d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
        s:      in  STD_LOGIC;
            y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component; 
  
  component registrador_n is 
    generic(constant N: integer := 8);
    port( clock, clear, enable: in STD_LOGIC;
        D: in STD_LOGIC_VECTOR(N-1 downto 0);
            Q: out STD_LOGIC_VECTOR (N-1 downto 0));
  end component;
  
  component hazarddec is
    port( branch, branch_id, branch_ex    : in  std_logic;
        pcsrc, pcsrc_mem          : in  std_logic;
      jump, jumpid, memwrite, alusrc, wid, wex  : in  std_logic;
      r1, r2, rid, rex          : in  std_logic_vector(4 downto 0);
      enablepc, flushid, enableif     : out std_logic);
  end component;
  
  signal writereg:           STD_LOGIC_VECTOR(4 downto 0);
  signal pcjump, pcnext, 
         pcnextbr, pcplus4, 
         pcbranch:           STD_LOGIC_VECTOR(31 downto 0);
  signal signimm, signimmsh: STD_LOGIC_VECTOR(31 downto 0);
  signal srca, srcb, result: STD_LOGIC_VECTOR(31 downto 0);
  signal s_aluout, s_writedata : std_logic_vector(31 downto 0);
  signal s_zero, pcsrc : std_logic;
  
  --sinais pos pipeline                        
  signal s_if  : std_logic_vector( 63 downto 0); 
  signal s_id  : std_logic_vector(142 downto 0);  
  signal s_ex  : std_logic_vector(106 downto 0);
  signal s_mem : std_logic_vector( 71 downto 0);
  
  signal s_rid, s_rex : std_logic_vector(4 downto 0);
  
  -- hazard control unity
  signal s_enable_pc, s_flush_id, s_enable_if, s_pcsrc_mem : std_logic; 
  signal s_control_id : std_logic_vector(9 downto 0);
  signal s_jump : std_logic;
begin
 
  pcbrmux: mux2 
    generic map(32) 
    port map(pcplus4, pcbranch, pcsrc, pcnextbr);  
    
  pcmux: mux2 
    generic map(32) 
    port map(pcnextbr, pcjump, s_jump, pcnext);
                                         
  pcreg: registrador_n
    generic map(32)
      port map( clk, reset, s_enable_pc,
              pcnext,
              pc);
    
  pcadd1: adder 
    port map(pc, X"00000004", pcplus4);
  
  ----------------------------------   
  -- PcPlus4 (63 downto 32) |Instruction (31 downto 0)
  if_reg : registrador_n
    generic map(64)
    port map( clk, reset, s_enable_if,
          pcplus4 & instr,
            s_if);
  ----------------------------------
  
  op    <= s_if(31 downto 26);
  funct <= s_if( 5 downto  0);
  
  wrmux: mux2 
    generic map(5) 
    port map( s_if(20 downto 16), 
          s_if(15 downto 11), 
          regdst, writereg);
          
  rf: regfile 
    port map( clk, s_mem(0),
          s_if(25 downto 21), s_if(20 downto 16),
          s_mem(5 downto 1), result, 
          srca, s_writedata); 
  
  -- Imediate
    se: signext 
      port map(s_if(15 downto 0),c, signimm);
    
  -- Jump  
  pcjump <= s_if(63 downto 60) & s_if(25 downto 0) & "00"; -- endereço jump 
  s_jump <= jump and not s_flush_id;
  
  --flush 
   flush_id_mux: mux2 
    generic map(10) 
    port map( jump & alusrc & alucontrol & s_if(26) & branch & memwrite & memtoreg & regwrite, 
          "00" & X"00", 
          s_flush_id, s_control_id); 
          
  ----------------------------------  
  -- jump (142) | alusource (141) | alucontrol (140 downto 138)||
  -- inst0 (137) | branch (136) | memwrite (135) || memtoreg (134)
  -- Pcplus4 (133 downto 102)|Reg1 (101 downto 70)|Reg2 (69 downto 38)|
  -- Imediate (37 dwonto 6)| WriteAddress (5 downto 1) | WriteEnable (0)
  
  id_reg : registrador_n
    generic map(143)
    port map( clk, reset, '1',
          s_control_id(9 downto 1) &
          s_if(63 downto 32) & srca & s_writedata & signimm & writereg & s_control_id(0),
            s_id);
  ----------------------------------
     
  -- PC Logic
  immsh: sl2 
    port map(s_id(37 downto 6), signimmsh);   
  
  pcadd2: adder 
    port map(s_id(133 downto 102), signimmsh, pcbranch);
      
  -- ALU   
  srcbmux: mux2 
    generic map(32) 
    port map(s_id(69 downto 38), s_id(37 downto 6), s_id(141),srcb);  
  
  mainalu: alu 
    port map(s_id(101 downto 70), srcb, s_id(140 downto 138), s_aluout, s_zero);
       
  ----------------------------------
  -- inst0 (106) | branch (105) | memwrite (104) || memtoreg (103)
  -- PcBranch (102 downto 71)| zero (70)| AluOut (69 downto 38)| Reg2 (37 downto 6)|
  -- WriteAddress (5 downto 1) | WriteEnable (0)
  ex_reg : registrador_n
    generic map(107)
    port map( clk, reset, '1',
          s_id(137 downto 134) & pcbranch & s_zero & s_aluout & s_id(69 downto 38) & s_id(5 downto 0),
            s_ex);
  ----------------------------------
                   
  memwritepip <= s_ex(104);
  pcsrc     <= s_ex(105) and (s_ex(70) xor s_ex(106));
  aluout    <= s_ex(69 downto 38);
  writedata   <= s_ex(37 downto 6);
        
  ----------------------------------
  -- pcsrc(71) | memtoreg (70)
  -- ReadData (69 downto 38) | AluOut (37 downto 6) |
  -- WriteAddress (5 downto 1) | WriteEnable (0)
  mem_reg : registrador_n
    generic map(72)
    port map( clk, reset, '1',
          pcsrc & s_ex(103) & readdata & s_ex(69 downto 38) & s_ex(5 downto 0),
            s_mem);
  ----------------------------------
  
  resmux: mux2 
    generic map(32) 
    port map( s_mem(37 downto 6), s_mem(69 downto 38), 
          s_mem(70), result); 
      
  ----------------------------------
  -- Hazard control unity
  
  haz : hazarddec
  port map(   branch, s_id(136), s_ex(105), 
          pcsrc, s_mem(71),
          jump, s_id(142), memwrite, alusrc, s_id(0), s_ex(0), 
          s_if(25 downto 21), s_if(20 downto 16), s_id(5 downto 1), s_ex(5 downto 1),
          s_enable_pc, s_flush_id, s_enable_if);  
        
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity alu is 
  port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
       alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
       result:     buffer STD_LOGIC_VECTOR(31 downto 0);
       zero:       out STD_LOGIC);
end;

architecture behave of alu is
  signal condinvb, sum: STD_LOGIC_VECTOR(31 downto 0);
begin
  condinvb <= not b when alucontrol(2) else b;
  sum <= a + condinvb + alucontrol(2);

  process(all) begin
    case alucontrol(1 downto 0) is
      when "00"   => result <= a and b; 
      when "01"   => result <= a or b; 
      when "10"   => result <= sum; 
      when "11"   => result <= (0 => sum(31), others => '0'); 
      when others => result <= (others => 'X'); 
    end case;
  end process;

  zero <= '1' when result = X"00000000" else '0';
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity regfile is -- three-port register file
  port(clk:           in  STD_LOGIC;
       we3:           in  STD_LOGIC;
       ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0);
       wd3:           in  STD_LOGIC_VECTOR(31 downto 0);
       rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of regfile is
  type ramtype is array (31 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
  signal mem: ramtype;
begin
  -- three-ported register file
  -- read two ports combinationally
  -- write third port on rising edge of clock
  -- register 0 hardwired to 0
  -- note: for pipelined processor, write third port
  -- on falling edge of clk
  process(clk) begin
    if falling_edge(clk) then
       if we3 = '1' then mem(to_integer(wa3)) <= wd3;
       end if;
    end if;
  end process;
  process(all) begin
    if (to_integer(ra1) = 0) then rd1 <= X"00000000"; -- register 0 holds 0
    else rd1 <= mem(to_integer(ra1));
    end if;
    if (to_integer(ra2) = 0) then rd2 <= X"00000000"; 
    else rd2 <= mem(to_integer(ra2));
    end if;
  end process;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity adder is -- adder
  port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
       y:    out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of adder is
begin
  y <= a + b;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity sl2 is -- shift left by 2
  port(a: in  STD_LOGIC_VECTOR(31 downto 0);
       y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of sl2 is
begin
  y <= a(29 downto 0) & "00";
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity signext is -- sign extender
  port(a: in  STD_LOGIC_VECTOR(15 downto 0);
       c: in  STD_LOGIC; -- c = '0' arit, c = '1' logical
       y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of signext is 
  signal s_0, s_1, s_temp : STD_LOGIC_VECTOR(31 downto 0);
begin  
  s_0 <= X"0000" & a;
  s_1 <= X"ffff" & a; 
  
  s_temp  <= s_1 when a(15) else s_0;
  y     <= s_0 when c else s_temp;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;

entity mux2 is -- two-input multiplexer
  generic(width: integer);
  port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:      in  STD_LOGIC;
       y:      out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture behave of mux2 is
begin
  y <= d1 when s else d0;
end;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;library IEEE;

entity hazarddec is
  port( branch, branch_id, branch_ex    : in  std_logic;
        pcsrc, pcsrc_mem          : in  std_logic;
      jump, jumpid, memwrite, alusrc, wid, wex  : in  std_logic;
      r1, r2, rid, rex          : in  std_logic_vector(4 downto 0);
      enablepc, flushid, enableif     : out std_logic);
end hazarddec;

architecture hazarddec_arc of hazarddec is
  signal s_r1, s_r2 : std_logic;      
  signal s_flush1 : std_logic;
  signal s_enable1, s_enable2 : std_logic;
  signal s_r1_rid, s_r1_rex, s_r2_rid, s_r2_rex, s_conflito : std_logic;
  signal s_r1_rid1, s_r1_rex1, s_r2_rid1, s_r2_rex1 : std_logic;
begin                         
  --conflito
  s_r1 <= not  jump;
  s_r2 <= not (jump or (alusrc and not memwrite));
  
  s_r1_rid1 <= '0' when r1 = "00000" else s_r1 and wid;
  s_r1_rex1 <= '0' when r1 = "00000" else s_r1 and wex;
  s_r2_rid1 <= '0' when r2 = "00000" else s_r2 and wid;
  s_r2_rex1 <= '0' when r2 = "00000" else s_r2 and wex;
    
  s_r1_rid <= s_r1_rid1 when r1 = rid else '0'; 
  s_r1_rex <= s_r1_rex1 when r1 = rex else '0'; 
  s_r2_rid <= s_r2_rid1 when r2 = rid else '0'; 
  s_r2_rex <= s_r2_rex1 when r2 = rex else '0';        
    
  s_conflito <= s_r1_rid or s_r1_rex or s_r2_rid or s_r2_rex;
  -----------------------------------------------------------     
  
  enablepc <= (not s_conflito) and (branch_ex or not(branch or branch_id));
  enableif <= (not s_conflito)  and ((branch_ex and not pcsrc) or not(branch or branch_id) or pcsrc_mem);
  flushid  <= jumpid or s_conflito or branch_id or branch_ex or pcsrc_mem;
end hazarddec_arc;

----------------------------------------------------------------------------------------------
library IEEE; 
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;
use STD.TEXTIO.all;
use IEEE.STD_LOGIC_ARITH.all;
entity registrador_n is
  generic (constant N: integer := 8);
  port (clock, clear, enable: in STD_LOGIC;
        D: in STD_LOGIC_VECTOR(N-1 downto 0);
        Q: out STD_LOGIC_VECTOR (N-1 downto 0));
end registrador_n;

architecture registrador_n of registrador_n is
  signal IQ: STD_LOGIC_VECTOR(N-1 downto 0); -- sinal Q interno
begin

process(clock, clear, enable, IQ)
  begin
    if (clear = '1') then IQ <= (others => '0');
    elsif (clock'event and clock='1') then
      if (enable='1') then IQ <= D; end if;
    end if;
    Q <= IQ;
  end process;
  
end registrador_n;

