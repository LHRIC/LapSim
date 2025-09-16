from utility.read_xlsx import read_xlsx
hdpt = read_xlsx('parameters/HDPT_Export.xlsx')
print(hdpt['front_left'].keys())
print(hdpt['rear_left'].keys())