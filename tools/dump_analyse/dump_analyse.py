#!/usr/bin/env python3
import sys
import re

def get_dump_file():
    return sys.argv[1]

def get_sym_file():
    return sys.argv[2]

def output_call_stack_file():
    return sys.argv[3]

def get_data_need_check(str):
    data = re.split(':', str)
    return data[3].strip()

def transfer_sym_file_to_dict(sym_file_name):
    sym_dict = dict()
    try:
        f = open(sym_file_name)
        while True:
            line = f.readline()
            if len(line) == 0:
                break
            line = re.split('\s', line.strip())
            if line[len(line)-2] == 't' or line[len(line)-2] == 'T':
                sym_dict[line[0]] = line[len(line)-1]

    except IOError:
        print("Could not find file %s"%sym_file_name)
    except KeyboardInterrupt:
        print("You cancelled the reading from the file")
    finally:
        if f:
            f.close()
        return sym_dict

dump_stack  = get_dump_file()
sym_file    = get_sym_file()
output_file = output_call_stack_file()
print("====================files need focus=================")
print("***********:dump_stack  %s."%(dump_stack))
print("***********:sym_file    %s."%(sym_file))
print("***********:output_file %s."%(output_file))
print("=====================================================")

sym_dict = transfer_sym_file_to_dict(sym_file)

try:
    stack_file = open(output_file, 'w')
    dump_file = open(dump_stack)
    while True:
        line = dump_file.readline()
        if len(line) == 0:
            break
        if len(line) != 115:
            stack_file.write(line)
            continue
        stack_file.write(line)
        line_content = get_data_need_check(line)
        line_content = re.split('\s', line_content)
        sym_dict_key_list_new = sym_dict.keys()
        sym_dict_key_list = []
        sym_dict_key_list = sorted(sym_dict_key_list_new)

        for item in line_content:
            i = 0
            while(i < len(sym_dict_key_list)):
                if item > sym_dict_key_list[-1]:
                    break
                if ((i == len(sym_dict_key_list)-1) and (item >= sym_dict_key_list[i])):
                    if sym_dict[sym_dict_key_list[i]] == "_vectors":
                        break
                    str1 = str(item)+': '+str(sym_dict[sym_dict_key_list[i]])
                    str1 = "               "+str1
                    stack_file.write(str1+'\n')
                    break
                elif ((item >= sym_dict_key_list[i]) and (item < sym_dict_key_list[i+1])):
                    if sym_dict[sym_dict_key_list[i]] == "_vectors":
                        break
                    str1 = str(item)+': '+str(sym_dict[sym_dict_key_list[i]])
                    str1 = "               "+str1
                    stack_file.write(str1+'\n')
                    break
                else:
                    i = i + 1

except IOError:
    print("Could not find file %s"%dump_stack)
except KeyboardInterrupt:
    print("You cancelled the reading from the file.!!!")
finally:
    if dump_file:
        dump_file.close()
