-- VHDL Beispiel
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

ENTITY SN74x6541 IS
  PORT(
    OE1a_n,OE1b_n : IN  STD_LOGIC;
    OE2a_n,OE2b_n : IN  STD_LOGIC;
    A1,A2:          IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
    Y1,Y2:          OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
  );
END SN74x6541;

ARCHITECTURE arch OF SN74x6541 IS
BEGIN
  Y1 <= A1 WHEN OE1a_n = '0' AND OE1b_n='0' ELSE
        (OTHERS=>'Z');
  Y2 <= A1 WHEN OE2a_n = '0' AND OE2b_n='0' ELSE
        (OTHERS=>'Z');        
END arch;
