# for generate the order.txt

from ast import Num
import os
import random
import re

folder_path = '../json'

# read json name
# file_names = sorted(os.listdir(folder_path))
# output_file_path = './order_list.txt'
# with open(output_file_path, 'w') as f:
#     for file_name in file_names:
#         print(file_name)
#         f.write(file_name + '\n')


# get the json number
file_list = os.listdir(folder_path)
file_count = len(file_list)

dt = dict()
try:
    with open("pos.txt", "r") as file:
        for line in file:
            line = line.strip()  # 去除行尾的換行符和空格
            number = re.findall(r'\d+', line)
            nums = [int(num) for num in number]
            
            dt[nums[0]] = (nums[1], nums[2])
            #print(nums)  # 印出每一行的內容
except:
    print("No pos.txt file")

#print(dt[21][0])

numbers = list(range(1, file_count+1))
random.shuffle(numbers)
output_file_path = './order.txt' 

with open(output_file_path, 'w') as f:
    for i, num in enumerate(numbers):
        #print(num)
        in_dic = dt.get(num)
        if i+1 == len(numbers):
            if in_dic is not None:
                f.write(str(num) + ' ' + str(dt[num][0]) + ' ' + str(dt[num][1]))
            else:
                f.write(str(num))
        else:
            if in_dic is not None:
                f.write(str(num) + ' ' + str(dt[num][0]) + ' ' + str(dt[num][1]) + '\n')
            else:
                f.write(str(num) + '\n')


# #print(numbers)
# random.shuffle(numbers)
# #print(numbers)
# output_file_path = './order.txt'
# with open(output_file_path, 'w') as f:
#     for i, num in enumerate(numbers):
#         #print(num)
#         if i+1 == len(numbers):
#             f.write(str(num) + ' ' + str(dt[num]))
#         else:
#             f.write(str(num) + '\n')