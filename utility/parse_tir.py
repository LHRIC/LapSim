import string

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
                if value.lstrip('-').replace('.', '', 1).isdigit():
                    value = float(value) if '.' in value else int(value)

                parsed_data[key.strip()] = value

    return parsed_data
