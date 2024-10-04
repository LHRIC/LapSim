import yaml
import pint

def read_yaml(file:str):
    with open(file,'r') as input:
        data = yaml.safe_load(input)
    return data

def convert(params:dict):
    ureg = pint.UnitRegistry(autoconvert_to_preferred=False)
    ureg.default_preferred_units = [
        ureg.meter,
        ureg.kilogram,
        ureg.second,
        ureg.newton
    ]

    kgms_params_list = []
    print(params)
    for key, value in params.items():
        value_adjusted = ureg.Quantity(value)
        value_adjusted = value_adjusted.to_preferred()
        print(f'{key}: {ureg.Quantity(value)} -> {value_adjusted}')
        kgms_params_list.append((key,value_adjusted.magnitude))
        kgms_params = dict(kgms_params_list)
    return kgms_params