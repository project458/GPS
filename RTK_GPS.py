import serial, pynmea2, ERP42
import math
from haversine import haversine
import numpy as np
from matplotlib import pyplot as plt
from threading import Thread

lat1 = 0;long1 = 0;gga_lon = 0;gga_lat = 0;car_go = 0.0;steer = 0;angle = 0;bear = 0.0
park_trigger = 0;child_trigger = 0;corner_trigger = 0;cross_trigger = 0;obstacle_trigger = 0;esp_trigger = 0
traffic_straight_trigger = 0;traffic_left_trigger = 0;big_obstacle_trigger = 0;cross2_trigger = 0
slow_velocity_trigger = 0
wp_interval = 2  # WP 간격
corner_interval = 2  # corner 포인트 간격
as_of_corner_angle = 3.0  # 코너 산정 기준
speed_parameter = 0
brake_parameter = 0
s_angle = 0 #
search_key = True # True 이면 도중 출발을 위한 인덱스 찾기 // False 이면 인덱스 0에서 출발
brake_key = False
code_end_key = False
corner_key = True;child_key = True;park_key = True;obstacle_key = True;big_obstacle_key = True;esp_key = True
cross_key = True;cross2_key = True;traffic_straight_key = True;traffic_left_key = True; slow_velocity_key = True
# GPS 데이터 파일 경로
path = 'k_section/k_course'
file_path = path + '.txt'
corner_path = path + '_corner.txt'
child_path = path + '_child.txt'
park_path = path + '_park.txt'
obstacle_path = path + '_ob1.txt'
big_obstacle_path = path + '_ob2.txt'
esp_path = path + '_esp.txt'
traffic_straight_path = path + '_ts.txt'
traffic_left_path = path + '_tl.txt'
slow_velocity_path = path + '_sv.txt'



# 스티어링 각 계산 함수
def steer(angle, go):
    s = angle - go

    if (s > 180):
        return s - 360
    elif (s < -180):
        return s + 360
    else:
        return s


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


# 코너 찾기 함수
def corner_search(pot):
    global corner_interval, as_of_corner_angle
    index = 0
    spot = process(pot, corner_interval)
    corner = []
    for i in range(len(spot)):
        wp1 = abs(Bearing(spot[index][0], spot[index][1], spot[index + 1][0], spot[index + 1][1]))
        wp2 = abs(Bearing(spot[index][0], spot[index][1], spot[index + 2][0], spot[index + 2][1]))

        # 코너 판단 기준
        if abs(wp1 - wp2) >= as_of_corner_angle:
            # print(wp1, wp2)
            corner.append((spot[index][0], spot[index][1]))
            corner.append((spot[index + 1][0], spot[index + 1][1]))

            # print('Coner Index: ',index)
            index += 1
        if index + 2 >= len(spot) - 1:
            break
        else:
            index += 1
    # print(corner)
    return corner


# 도중 출발을 위한 출발 인덱스 부여 함수
def search_start_location(cur_lat, cur_lon):
    global pot, wp_interval, search_key
    spot = process(pot, wp_interval)  # 2m 간격의 WP 경로 생성
    while 1:
        if cur_lat == 0 and cur_lon == 0:
            continue
        for i in range(len(spot) - 1):
            dis = distance((cur_lat, cur_lon), (spot[i][0], spot[i][1]))
            c_vec = Bearing(cur_lat, cur_lon, spot[i][0], spot[i][1])  # 현재 위치에서 목표 좌표 방향
            t_vec = Bearing(spot[i][0], spot[i][1], spot[i + 1][0], spot[i + 1][1])  # 목표 좌표에서 다음 목표 좌표 방향
            if dis <= wp_interval + 2 and abs(c_vec - t_vec) <= 40:  # 비교
                start_index = i
                search_key = False
                #print('start_searching(%d)' % (i))
                return start_index
        return 0


# 도중 출발을 위한 트리거 인덱스 부여 함수
def search_trigger_index(start_index, trigger_list_name):
    global pot, wp_interval, search_key
    spot = process(pot, wp_interval)  # 2m 간격의 WP 경로 생성
    while 1:
        for i in range(len(trigger_list_name)):
            if start_index <= spot.index(trigger_list_name[i]):  # 출발 인덱스와 트리거 인덱스를 전체 경로 인덱스에서 비교
                #print('trigger_searching(%d)' % (spot.index(trigger_list_name[i])))
                return i
        return 0


# GPS 데이터 시각화 함수
def GPS_plot():
    global pot1, gga_lat, gga_lon, gps
    gps = np.array(pot1)

    try:
        while 1:
            plt.plot(gps[:, 1], gps[:, 0], '.', c='r')
            plt.axis('scaled')
            plt.xticks(color='w')
            plt.yticks(color='w')
            plt.plot(gga_lon, gga_lat, '.', c='b')
            plt.draw()
            plt.pause(0.01)
            plt.cla()

    except:
        pass


def GPS_Drive():
    global lat1, long1, gga_lat, gga_lon, car_go, pot1, angle, bear, speed_parameter, brake_parameter
    global search_key, brake_key, code_end_key, traffic_straight_key, traffic_left_key, slow_velocity_key
    global corner_key, park_key, child_key, obstacle_key, big_obstacle_key, esp_key
    global park_pot, corner_pot, child_pot, obstacle_pot, esp_pot, traffic_straight_pot, traffic_left_pot
    global big_obstacle_pot, slow_velocity_pot, slow_velocity_trigger, s_angle
    global park_trigger, child_trigger, corner_trigger, cross_trigger, obstacle_trigger, esp_trigger
    global traffic_straight_trigger, traffic_left_trigger, big_obstacle_trigger, cross2_trigger, gps_braking

    ser = serial.Serial('com8', 460800, timeout=10.0)                   # 시리얼 통신
    print("GPS connected to " + ser.portstr)                            # 연결된 GPS 포트 확인

    start = 0                                                           # 출발 스팟 인덱싱
    end = len(pot1) - 1                                                 # 도착 스팟 인덱싱
    park_start = 0                                                      # 주차 시작 스팟 인덱싱
    park_end = len(park_pot) - 1                                        # 주차 종료 스팟 인덱싱
    child_start = 0                                                     # 어린이 보호 구역 시작 스팟 인덱싱
    child_end = len(child_pot) - 1                                      # 어린이 보호 구역 스팟 인덱싱
    obstacle_start = 0                                                  # 장애물 스팟 인덱싱
    obstacle_end = len(obstacle_pot) - 1                                # 장애물 스팟 인덱싱
    esp_start = 0                                                       # 긴급정지 시작 스팟 인덱싱
    esp_end = len(esp_pot) - 1                                          # 긴급정지 시작 스팟 인덱싱
    traffic_straight_start = 0                                          # 신호등 시작 스팟 인덱싱
    traffic_straight_end = len(traffic_straight_pot) - 1                # 신호등 시작 스팟 인덱싱
    corner_start = 0                                                    # 코너 시작 스팟 인덱싱
    corner_end = len(corner_pot) - 1                                    # 코너 종료 스팟 인덱싱
    traffic_left_start = 0
    traffic_left_end = len(traffic_left_pot) - 1
    big_obstacle_start = 0
    big_obstacle_end = len(big_obstacle_pot) - 1
    slow_velocity_start = 0
    slow_velocity_end = len(slow_velocity_pot) - 1
    gps_braking = 0                                                     # gps 코드 끝남 및 브레이크 신호 알려주는 트리거

    '''
    k_key = input('도중 출발입니까? Yes(1) No(0)\n')  # input()는 문자형으로 저장한다
    
    if k_key == '1':
        search_key = True
    '''

    while 1:
        try:
            speed_parameter = 7  # 직진 구간일  때 ERP42 speed parameter
            if ser.readable():
                line = ser.readline().decode("utf-8")[:-2]  # 시리얼 통신을 통해 받은 데이터 읽은 후 디코딩

                # GPS 데이터 파싱
                msg = pynmea2.parse(line)

                # 데이터 형식이 RMC일 때
                if msg.sentence_type == 'RMC' and msg.status == 'A':
                    gga_lat = msg.latitude  # 위도 저장
                    gga_lon = msg.longitude  # 경도 저장

                    bear = Bearing(gga_lat, gga_lon, pot1[start][0], pot1[start][1])  # 현재 위치에서 목표 위치 방향 계산

                    if search_key:  # 도중 출발 키 True
                        start = search_start_location(gga_lat, gga_lon)  # 시작 스팟 인덱싱
                        park_start = search_trigger_index(start, park_pot)
                        corner_start = search_trigger_index(start, corner_pot)
                        esp_start = search_trigger_index(start, esp_pot)
                        obstacle_start = search_trigger_index(start, obstacle_pot)
                        big_obstacle_start = search_trigger_index(start, big_obstacle_pot)
                        traffic_straight_start = search_trigger_index(start, traffic_straight_pot)
                        traffic_left_start = search_trigger_index(start, traffic_left_pot)
                        child_start = search_trigger_index(start, child_pot)
                        slow_velocity_start = search_trigger_index(start,slow_velocity_pot)
                        print('Search Complete')

                    lat1 = pot1[start][0]  # 목표 위도
                    long1 = pot1[start][1]  # 목표 경도

                    if msg.true_course is None:  # 헤딩값 들어오지 않을 시 방위각 부여
                        car_go = bear
                    else:
                        car_go = msg.true_course

                # 플랫폼에서 일정 거리 이상인 WP를 목표 지점으로 갱신
                if not code_end_key:
                    if distance((gga_lat, gga_lon), (pot1[start][0], pot1[start][1])) < 3.0:
                        start += 1
                        if start == end:
                            # 마지막 WP 도달 시 ERP42 브레이크 작동 및 정지
                            if distance((gga_lat, gga_lon), (pot1[start][0], pot1[end][1])) < 2.0:
                                code_end_key = True
                                brake_parameter = 33
                                gps_braking = 1
                            else:
                                start -= 1

                # 주차 구역 가기 전 속도 10km/h 구간
                if distance((gga_lat, gga_lon),
                            (slow_velocity_pot[slow_velocity_start][0],
                             slow_velocity_pot[slow_velocity_start][1])) < 3.0 and slow_velocity_key:
                    slow_velocity_trigger = True
                    print('Slow Velocity Area')
                    slow_velocity_start += 1
                    if slow_velocity_start == slow_velocity_end:
                        if distance((gga_lat, gga_lon),
                                    (slow_velocity_pot[slow_velocity_end][0],
                                     slow_velocity_pot[slow_velocity_end][1])) < 2.0:
                            slow_velocity_trigger = False
                            slow_velocity_key = False
                        else:
                            slow_velocity_start -= 1

                # 주차 구역일 때 트리거 발생:
                if distance((gga_lat, gga_lon), (park_pot[park_start][0], park_pot[park_start][1])) < 3.0 and park_key:
                    speed_parameter = 3
                    park_trigger = True
                    print('Park Area')
                    park_start += 1
                    if park_start == park_end:
                        if distance((gga_lat, gga_lon), (park_pot[park_end][0], park_pot[park_end][1])) < 2.0:
                            park_trigger = False
                            park_key = False
                        else:
                            park_start -= 1
                
                # 코너 구간일 때 트리거 발생
                if distance((gga_lat, gga_lon),
                            (corner_pot[corner_start][0], corner_pot[corner_start][1])) < 7.0 and corner_key:
                    corner_trigger = True
                    speed_parameter=5
                    print('Corner Area')
                    corner_start += 1
                    if corner_start == corner_end:
                        if distance((gga_lat, gga_lon),
                                    (corner_pot[corner_end][0], corner_pot[corner_end][1])) < 2.0:
                            corner_trigger = False
                            corner_key = False
                        else:
                            corner_start -= 1
                if abs(Bearing(gga_lat,gga_lon,corner_pot[corner_start-1][0], corner_pot[corner_start-1][1])-bear) > 50:
                    corner_trigger = False

                # 어린이 보호 구역일 때 트리거 발생
                if distance((gga_lat, gga_lon),
                            (child_pot[child_start][0], child_pot[child_start][1])) < 3.0 and child_key:
                    speed_parameter = 5
                    child_trigger = True
                    print('Child Area')
                    child_start += 1
                    if child_start == child_end:
                        if distance((gga_lat, gga_lon), (child_pot[child_end][0], child_pot[child_end][1])) < 2.0:
                            child_trigger = False
                            child_key = False
                        else:
                            child_start -= 1

                # 장애물(소형) 구역일 때 트리거 발생
                if distance((gga_lat, gga_lon),
                            (obstacle_pot[obstacle_start][0], obstacle_pot[obstacle_start][1])) < 3.0 and obstacle_key:
                    obstacle_trigger = True
                    print('Obstacle Area')
                    obstacle_start += 1
                    if obstacle_start == obstacle_end:
                        if distance((gga_lat, gga_lon),
                                    (obstacle_pot[obstacle_end][0], obstacle_pot[obstacle_end][1])) < 2.0:
                            obstacle_trigger = False
                            obstacle_key = False
                        else:
                            obstacle_start -= 1

                # 장애물(대형) 구역일 때 트리거 발생
                if distance((gga_lat, gga_lon),
                            (big_obstacle_pot[big_obstacle_start][0],
                             big_obstacle_pot[big_obstacle_start][1])) < 3.0 and big_obstacle_key:
                    big_obstacle_trigger = True
                    print('Big Obstacle Area')
                    big_obstacle_start += 1
                    if big_obstacle_start == big_obstacle_end:
                        if distance((gga_lat, gga_lon),
                                    (big_obstacle_pot[big_obstacle_end][0],
                                     big_obstacle_pot[big_obstacle_end][1])) < 2.0:
                            big_obstacle_trigger = False
                            big_obstacle_key = False
                        else:
                            big_obstacle_start -= 1

                # 긴급 정지 구역일 때 트리거 발생
                if distance((gga_lat, gga_lon), (esp_pot[esp_start][0], esp_pot[esp_start][1])) < 3.0 and esp_key:
                    esp_trigger = True
                    print('Esp Area')
                    esp_start += 1
                    if esp_start == esp_end:
                        if distance((gga_lat, gga_lon), (esp_pot[esp_end][0], esp_pot[esp_end][1])) < 2.0:
                            esp_trigger = False
                            esp_key = False
                        else:
                            esp_start -= 1

                # 신호등 직진 구역일 때 트리거 발생
                if distance((gga_lat, gga_lon), (traffic_straight_pot[traffic_straight_start][0],
                                                 traffic_straight_pot[traffic_straight_start][1])) < 3.0 and \
                        traffic_straight_key:
                    traffic_straight_trigger = True
                    print('Traffic Straight Area')
                    traffic_straight_start += 1
                    if traffic_straight_start == traffic_straight_end:
                        if distance((gga_lat, gga_lon), (traffic_straight_pot[traffic_straight_end][0],
                                                         traffic_straight_pot[traffic_straight_end][1])) < 2.0:
                            traffic_straight_trigger = False
                            traffic_straight_key = False
                        else:
                            traffic_straight_start -= 1
                if abs(Bearing(gga_lat,gga_lon,traffic_straight_pot[traffic_straight_start-1][0],
                                traffic_straight_pot[traffic_straight_start-1][1])-bear) > 50:
                        traffic_straight_trigger = False   

                # 신호등 좌회전 구역일 때 트리거 발생
                if distance((gga_lat, gga_lon),
                            (traffic_left_pot[traffic_left_start][0],
                             traffic_left_pot[traffic_left_start][1])) < 3.0 and traffic_left_key:
                    traffic_left_trigger = True
                    print('Traffic Left Area')
                    traffic_left_start += 1
                    if traffic_left_start == traffic_left_end:
                        if distance((gga_lat, gga_lon),
                                    (traffic_left_pot[traffic_left_end][0],
                                     traffic_left_pot[traffic_left_end][1])) < 2.0:
                            traffic_left_trigger = False
                            traffic_left_key = False
                        else:
                            traffic_left_start -= 1
                if abs(Bearing(gga_lat,gga_lon,traffic_left_pot[traffic_left_start-1][0],
                                             traffic_left_pot[traffic_left_start-1][1])-bear) > 50:
                    traffic_left_trigger = False    

                angle = (steer(bear, car_go) - ERP42.STEER()) * 0.5  # ERP42가 조향해야할 각 계산
                s_angle = steer(bear, car_go)                        # 수원이 에게 넘겨주는 차량 목표 각도
                '''
                # ERP42 Control Function
                if code_end_key:                                    # 코드 종료 키 -> 주행 코드 종료
                    ERP42.BRAKE(brake_parameter)
                    print('BRAKE ERP42 and code fin')
                    break
                if brake_key:                                       # 브레이크 키 -> ERP42 브레이크 명령
                    ERP42.BRAKE(brake_parameter)
                    brake_key = False
                else:                                               # 그 이외일 경우, ERP42 주행 명령
                    ERP42.GEAR(0)
                    ERP42.SPEED(speed_parameter)
                    ERP42.STEER(angle)
                '''
                ser.flushInput()  # flush input buffer

        except serial.SerialException as e:  # 시리얼 포트 연결 안 되어 있을 때
            print('Device error: {}'.format(e))
            # break
            pass
        except pynmea2.ParseError as e:  # 파싱 오류
            print('Parse error: {}'.format(e))
            pass
        except UnicodeDecodeError as e:  # 디코딩 오류
            print('Decode error: {}'.format(e))
            pass


########################################################################################################################
########################################################################################################################


pot = GPS_Data_Read(file_path)  # GPS 데이터 파일 읽어보기 함수 사용


corner_pot = GPS_Data_Read(corner_path)  # GPS 코너 데이터 읽어보기 함수 사용
park_pot = GPS_Data_Read(park_path)  # GPS 주차 데이터 읽어보기 함수 사용
child_pot = GPS_Data_Read(child_path)
obstacle_pot = GPS_Data_Read(obstacle_path)
big_obstacle_pot = GPS_Data_Read(big_obstacle_path)
esp_pot = GPS_Data_Read(esp_path)
traffic_straight_pot = GPS_Data_Read(traffic_straight_path)
traffic_left_pot = GPS_Data_Read(traffic_left_path)
slow_velocity_pot = GPS_Data_Read(slow_velocity_path)


pot1 = process(pot, wp_interval)  # WP 만들기
gps = np.array(pot1)


########################################################################################################################
########################################################################################################################


# GPS 기반 주행 함수 // Driving using GPS threading start
GPS_drive_thread = Thread(target=GPS_Drive)
GPS_drive_thread.start()


# 경로 및 현재 위치 출력 // plot function
GPS_plot_Thread = Thread(target=GPS_plot)
#GPS_plot_Thread.start()

########################################################################################################################
########################################################################################################################