import pandas as pd
import src.type
reram_read = 15 # ns
reram_write = 1 # ns

dram_pim = 4.5 # ns
n_mac = 16

L = 768

dram_pim_time = L/n_mac*dram_pim

m = 8
n = 7
reram_pim_time = (12*n+6.5*m*m-7.5*m-2)*reram_write

print("DRAM PIM time (ns): ", dram_pim_time)
print("ReRAM PIM time (ns): ", reram_pim_time)

