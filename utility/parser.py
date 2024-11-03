import yaml
import pint
import string

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
        ureg.newton,
        ureg.radian, #this does nothing
    ]

    kgms_params_list = []
    print(params)
    for key, value in params.items():
        try:
            value_raw = ureg.Quantity(value)
            value_adjusted = value_raw.to_preferred()
            if value_raw.units==('degree'): #TODO make more robust
                value_adjusted=value_raw.to('radian')
            print(f'{key}: {ureg.Quantity(value_raw)} -> {value_adjusted}')
            kgms_params_list.append((key,value_adjusted.magnitude))
        except:
            kgms_params_list.append((key,value))
    kgms_params = dict(kgms_params_list)
    print(kgms_params)
    return kgms_params # FYI | deg -> rad | rpm -> rad/s 

def parse_tir(filepath):
    parsed_data = {}

    with open(filepath, "r") as file:
        for line in file:
            if '=' in line:
                # Split key value pair on = 
                key, value = line.strip().split('=', 1)
                value = value.strip()

                # Remove everything after $
                value = value.split('$', 1)[0].strip()

                # Convert value to int or float if possible
                if value.replace('.', '', 1).isdigit():
                    value = float(value) if '.' in value else int(value)

                parsed_data[key.strip()] = value

    return parsed_data