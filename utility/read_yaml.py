import yaml

def read_yaml(file:str):
    with open(file,'r') as input:
        data = yaml.safe_load(input)
    return data

hdpts = read_yaml('hardpoints.yaml')
rear_left = hdpts['rear_left']