import pandas as pd
import subprocess
import math
import os
from src.config import *
from src.model import *
from src.type import *


class NeuroSim:

    def __init__(self,
                 modelinfos,
                 neurosim_dir,
                 output_log='',
                 fast_mode=False,
                 num_chip=5):
        self.df = pd.DataFrame()
        self.neurosim_dir = neurosim_dir
        self.output_log = output_log
        if os.path.exists(output_log):
            self.df = pd.read_csv(output_log)
        self.tCK = 0.769  # ns
        self.num_chip = num_chip
        self.nhead = modelinfos['num_heads']
        self.dhead = modelinfos['dhead']
        self.fast_mode = fast_mode


    def update_log_file(self, log):
        if self.df.empty:
            if os.path.exists(self.output_log):
                df = pd.read_csv(self.output_log)
            else:
                columns = [
                    'L', 'nhead', 'dhead', 'dbyte', 'pim_type',
                    'power_constraint', 'time', 'energy'
                ]
                df = pd.DataFrame(columns=columns)
        else:
            df = self.df
        if len(df.columns) > 12:
            import pdb
            pdb.set_trace()
        new_df = pd.DataFrame(columns=df.columns)
        new_df.loc[0] = log
        df = pd.concat([df, new_df]).drop_duplicates()
        self.df = df
        self.df.to_csv(self.output_log, index=False)

    #def run_ramulator(self):
    def run_neurosim(self, pim_type: PIMType, l, num_ops_per_hbm, dbyte, file_name):
        pim_type_name = pim_type.name.lower(
        ) if not pim_type == PIMType.DIG else "digital"
        
        output_file = file_name + '.csv'


        # run neuroSim

        neurosim_exc = os.path.join(
            self.neurosim_dir,"main")
        
        # trace_args = "--dhead {} --nhead {} --seqlen {} --dbyte {} --output {}".format(
        #     self.dhead, num_ops_per_hbm, l, dbyte, output_file)

        debug = 0 #该参数用于控制neuroSim不输出无效信息

        neuorsim_args = "{} {} {} {} {} {}".format(
            self.dhead, num_ops_per_hbm, l, dbyte, output_file, debug)

        run_neurosim_cmd = f"{neurosim_exc} {neuorsim_args}"
        try:
            result = subprocess.run(run_neurosim_cmd,
                                    stdout=subprocess.PIPE,
                                    text=True,
                                    shell=True)
            output_lines = result.stdout.strip().split('\n') 
            output_list = [line.strip() for line in output_lines]
        except subprocess.CalledProcessError as e:
            print(f"Error: {e}")
            assert 0


        # parsing output

        # 单位 s pJ        
        time = 0.
        energy = 0.

        for line in output_list:
            if "totallatency" in line:
                time += float(line.split()[-1])
            elif "totalenergy" in line:
                energy += float(line.split()[-1])

        out = [time, energy]
        return out

    def run(self, pim_type: PIMType, layer: Layer, power_constraint=True):
        if os.path.exists(self.neurosim_dir):
            l = layer.n
            dhead = self.dhead
            dbyte = layer.dbyte
            num_ops_per_neurosim = layer.numOp #TODO 该部分实现有问题，numop的含义模糊
            num_ops_per_chip = math.ceil(num_ops_per_neurosim / self.num_chip)
            num_ops_group = 1
            if self.fast_mode:
                minimum_heads = 64
                num_ops_group = math.ceil(num_ops_per_chip / minimum_heads)
                num_ops_per_chip = minimum_heads

            file_name = "neurosim_l{}_nattn{}_dhead{}_dbyte{}_pc{}".format(
                l, num_ops_per_chip, dhead, layer.dbyte, int(power_constraint))
            
            # yaml_file = os.path.join(self.neurosim_dir, file_name + '.yaml')
            # self.make_yaml_file(yaml_file, file_name, power_constraint)

            result = self.run_neurosim(pim_type, l, num_ops_per_chip, layer.dbyte, file_name)
            # 返回结果修改为time energy
            # post processing


            log = [
                l, num_ops_per_chip, dhead, dbyte, pim_type.name,
                power_constraint
            ] + result
            self.update_log_file(log)
            
            exec_time, energy = result

            return exec_time, energy

        else:
            assert 0, "Need to install ramulator"
            return 0.0, 0.0

    def output(self, pim_type: PIMType, layer: Layer, power_constraint=True):
        if self.df.empty:
            self.run(pim_type, layer, power_constraint)

        num_ops_per_neurosim = layer.numOp
        num_ops_per_chip = math.ceil(num_ops_per_neurosim / self.num_chip)
        num_ops_group = 1
        if self.fast_mode:
            minimum_heads = 64
            num_ops_group = math.ceil(num_ops_per_chip / minimum_heads)
            num_ops_per_chip = minimum_heads

        l = layer.n
        dhead = layer.k
        dbyte = layer.dbyte
        row = self.df[(self.df['L'] == l) & (self.df['nhead'] == num_ops_per_chip) & \
                      (self.df['dbyte'] == dbyte) & (self.df['dhead'] == dhead) & \
                      (self.df['power_constraint'] == power_constraint) &  \
                      (self.df['pim_type'] == pim_type.name)]
        if row.empty:
            return self.run(pim_type, layer, power_constraint)

        else:
            time = float(row.iloc[0]['time'])
            energy = float(row.iloc[0]['energy'])

            return time, energy
