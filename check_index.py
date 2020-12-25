"""To check index of Way Point list"""
import numpy as np
import math
from haversine import haversine
from matplotlib import pyplot as plt

common_path = 'C:/Users/...'

# GPS 데이터 파일 경로
file_path = common_path + 'file_name'

sv_pot_path = common_path + 'file_name'
park_pot_path = common_path + 'file_name'
corner_pot_path = common_path + 'file_name'
child_pot_path = common_path + 'file_name'
esp_pot_path = common_path + 'file_name'
ob1_pot_path = common_path + 'file_name'
ob2_pot_path = common_path + 'file_name'
tl_pot_path = common_path + 'file_name'
ts_pot_path = common_path + 'file_name'

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

fig,ax = plt.subplots()

wp_interval = 2

pot = GPS_Data_Read(file_path)  # GPS 데이터 파일 읽어보기 함수 사용
pot1 = process(pot, wp_interval)  # WP 만들기
gps = np.array(pot1)

##### 완성된 구간 WP 파일 읽기 ##################
park_pot = np.array(GPS_Data_Read(park_pot_path))
sv = np.array(GPS_Data_Read(sv_pot_path))
corner_pot = np.array(GPS_Data_Read(corner_pot_path))
esp_pot = np.array(GPS_Data_Read(esp_pot_path))
ob1_pot = np.array(GPS_Data_Read(ob1_pot_path))
ob2_pot = np.array(GPS_Data_Read(ob2_pot_path))
sv_pot = np.array(GPS_Data_Read(sv_pot_path))
tl_pot = np.array(GPS_Data_Read(tl_pot_path))
ts_pot = np.array(GPS_Data_Read(ts_pot_path))
child_pot = np.array(GPS_Data_Read(child_pot_path))
plt.plot(gps[:, 1], gps[:, 0], '.', c='r') # 경로

def pot_index_check(pot):
    for i in range (len(pot)-1):
        count = i%5+1
        
        if count == 1:
            plt.plot(pot[i,1],pot[i, 0],'*',c='orange')
        if count == 2:
            plt.plot(pot[i, 1], pot[i, 0], '*', c='black')
        if count == 3:
            plt.plot(pot[i, 1], pot[i, 0], '*', c='purple')
        if count == 4:
            plt.plot(pot[i, 1], pot[i, 0], '*', c='green')
        if count == 5:
            plt.plot(pot[i, 1], pot[i, 0], '*', c='blue')

    plt.axis('scaled')
    plt.xticks(color='w')
    plt.yticks(color='w')
    plt.show()
'''
plt.plot(park_pot[:,1],park_pot[:,0],'*',c='orange')                # 주차 구역 WP 출력
plt.plot(corner_pot[:,1],corner_pot[:,0],'*',c='gold')              # 코너 구간 WP 출력
plt.plot(ob1_pot[:,1],ob1_pot[:,0],'*',c='black')                   # 소형 장애물 구간 WP 출력
plt.plot(ob2_pot[:,1],ob2_pot[:,0],'*',c='tan')                     # 대형 장애물 구간 WP 출력
plt.plot(child_pot[:,1],child_pot[:,0],'*',c='chocolate')           # 어린이 보호구역 WP 출력
plt.plot(esp_pot[:,1],esp_pot[:,0],'*',c='purple')                  # 긴급정지 구간 WP 출력
plt.plot(sv_pot[:,1],sv_pot[:,0],'*',c='navy')                      # 주차 구역 도달 전 속도 고정 구간 WP 출력
plt.plot(tl_pot[:,1],tl_pot[:,0],'*',c='green')                     # 좌회전 신호 구간 WP 출력
plt.plot(ts_pot[:,1],ts_pot[:,0],'*',c='blue')                      # 직진 신호 구간 WP 출력
'''

pot_index_check(corner_pot)