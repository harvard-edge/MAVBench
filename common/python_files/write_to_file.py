import sys
file_addr = sys.argv[1]
string_to_write = sys.argv[2]
with open(file_addr, 'a') as infile:
    infile.write(string_to_write)
