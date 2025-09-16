import pandas as pd

def read_xlsx(file:str):
    hdpt_fl = pd.read_excel(file,sheet_name='Hardpoint Configurator',
                         header=6,usecols="B:E",nrows=18)
    hdpt_dict_fl = hdpt_fl.set_index(hdpt_fl.columns[0]).T.to_dict('list')
    hdpt_rl = pd.read_excel(file,sheet_name='Hardpoint Configurator',
                         header=31,usecols="B:E",nrows=18)
    hdpt_dict_rl = hdpt_rl.set_index(hdpt_rl.columns[0]).T.to_dict('list')
    hdpt_dict = {"front_left": hdpt_dict_fl, "rear_left": hdpt_dict_rl}
    return hdpt_dict
