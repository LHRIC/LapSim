import yaml

def read_yaml(file:str):
    with open(file,'r') as input:
        data = yaml.safe_load(input)
    return data