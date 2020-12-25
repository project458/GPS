import numpy as np
import math, keyboard
from haversine import haversine
from matplotlib import pyplot as plt

## WP추가 리스트와 WP삭제 리스트와 별개의 리스트임 주의!!##

# GPS 데이터 파일 읽어보기 함수
def GPS_Data_Read(file_path):
    spot = []
    with open(file_path) as f:
        data = f.readlines()
        for i in data:
            a = i.strip().split(',')
            data_fin = (float(a[0]), float(a[1]))
            spot.append(data_fin)
    return spot

# 거리 계산 함수 (m)단위로 반환
def distance(a, b):
    return haversine(a, b) * 1000

# 방위각 계산 함수
def Bearing(lat1, long1, lat2, long2):
    Lat1, Lat2 = math.radians(lat1), math.radians(lat2)
    Long1, Long2 = math.radians(long1), math.radians(long2)
    y = math.sin(Long2 - Long1) * math.cos(Lat2)
    x = math.cos(Lat1) * math.sin(Lat2) - math.sin(Lat1) * math.cos(Lat2) * math.cos(Long2 - Long1)
    result = math.degrees(math.atan2(y, x)) + 360
    if result > 360:
        result = result - 360
        return result
    else:
        return result

# processing 함수
def process(pot, m):
    pot2 = []
    MJ_Spot = 0
    will_go = 1

    for i in range(len(pot)):
        if distance((pot[MJ_Spot][0], pot[MJ_Spot][1]), (pot[will_go][0], pot[will_go][1])) >= m:
            pot2.append((pot[will_go][0], pot[will_go][1]))
            MJ_Spot = will_go
            will_go += 1
        else:
            will_go += 1

        if will_go == len(pot) - 1:
            pot2.append((pot[will_go][0], pot[will_go][1]))
            break
    return pot2


#######################################################################################################################
#######################################################################################################################
wp_interval = 2
print('구간 수정 코드입니다.')
title = input('저장할 파일명(txt 제외)를 적어주십시오: ')



file_path = 'C:/Users/...'  # 전체 경로
modify_path = 'C:/Users/...'             # 수정하고 싶은 경로
compare_path = 'C:/Users/...'          # RTK GPS로 기록한 경로

pot = GPS_Data_Read(file_path)  # GPS 데이터 파일 읽어보기 함수 사용
pot1 = process(pot, wp_interval)  # WP 만들기
modify_list = GPS_Data_Read(modify_path) # 수정하고 싶은 파일 읽기
compare_list=GPS_Data_Read(compare_path)
copy_list = modify_list[:]

gps = np.array(pot1)
modify = np.array(modify_list)
compare = np.array(compare_list)

fig,ax = plt.subplots()
plt.plot(gps[:, 1], gps[:, 0], '.', c='r')              # WP 경로 생성
plt.plot(modify[:, 1], modify[:, 0], '^', c='c')        # 포인트 생성
plt.plot(compare[:,1],compare[:,0],'*',c='g')

plt.axis('scaled')
plt.xticks(color='w')
plt.yticks(color='w')

x_sub_data=[]
y_sub_data=[]

x_add_data = []
y_add_data = []

line, = ax.plot(x_sub_data,y_sub_data, '.', c='r')
aline, = ax.plot(x_add_data,y_add_data,'*',c='g')

def save_point(event):
    global line

    if event.inaxes != ax:
        print('영역 밖 클릭 함')
        return

    if event.button == 3 and keyboard.is_pressed('a'): # 마우스 우클릭 + 'a'키 누름 -> 클릭한 WP 추가할 예정
        x = round(float(event.xdata), 6)
        y = round(float(event.ydata), 6)
        for i in range(len(pot1)):
            if distance(pot1[i], (y, x)) <= 1.0:
                x_add_data.append(pot1[i][1])
                y_add_data.append(pot1[i][0])
                print('add')
                break

        aline.set_data(x_add_data,y_add_data)
        plt.draw()

    if event.button == 3: # 마우스 우클릭 -> 클릭한 WP 삭제할 예정
        x = round(float(event.xdata),6)
        y = round(float(event.ydata),6)
        for i in range(len(pot1)):
            if distance(pot1[i], (y, x)) <= 1.0:
                if pot1[i] in copy_list:
                    x_sub_data.append(pot1[i][1])
                    y_sub_data.append(pot1[i][0])
                    print('sub')
                    break
                if pot1[i] not in copy_list and not keyboard.is_pressed('a'):
                    print('No WP to sub')

        line.set_data(x_sub_data,y_sub_data)
        plt.draw()

    # 삭제할 좌표 리스트의 마지막 원소 제거
    if event.button==1 and keyboard.is_pressed('e'): # 왼쪽 마우스 클릭 + 'e'키 누르기

        x_sub_data.pop()
        y_sub_data.pop()
        print('sub return')
        line.set_data(x_sub_data, y_sub_data)
        plt.draw()

    # 추가할 좌표 리스트의 마지막 원소 제거
    if event.button==1 and keyboard.is_pressed('d'): # 왼쪽 마우스 클릭 + 'd'키 누르기

        x_add_data.pop()
        y_add_data.pop()
        print('add return')
        aline.set_data(x_add_data, y_add_data)
        plt.draw()

    if event.button == 1 and keyboard.is_pressed('Esc'):
        plt.disconnect(cid)
        plt.close()



########################################################################################################################
########################################################################################################################


cid = plt.connect('button_press_event', save_point)
plt.show()

mf = open('C:/Users/...' +title+ '_m.txt', mode='wt', encoding='utf-8')
for i in range(len(x_sub_data)):                                                    # WP 삭제 처리
    copy_list.remove((y_sub_data[i],x_sub_data[i]))

count_index = []
for i in range(len(x_add_data)):                                                    # WP 추가할 좌표 정렬
    count_index.append(pot1.index((y_add_data[i],x_add_data[i])))
for i in range(len(copy_list)):
    count_index.append(pot1.index(copy_list[i]))

count_index.sort()                                                             # 전체 경로 인덱스 기준 정렬

for i in count_index:
    mf.write("%f, %f\n" %(pot1[i][0],pot1[i][1]))
mf.close()

print('done')
