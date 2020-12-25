"""Used RTK GPS called ublox C099-F9P """
import serial, pynmea2, keyboard

ser = serial.Serial('com9', 460800, timeout=5.0) # 시리얼 통신

title = input("GPS 데이터를 저장하기 위해 (.txt)를 제외한 파일명을 입력해 주세요: ")

fw = open('C:/Users/...' + title + '.txt', mode='at', encoding='utf-8')


while 1:
    try:
        if ser.readable():
            line = ser.readline() # 시리얼 통신을 통해 받은 데이터 읽기

            data = line.decode('utf-8')[:-2] # GPS 데이터 디코딩 후 저장
            msg = pynmea2.parse(data) # GPS 데이터 파싱
            #print(msg)
            if msg.sentence_type == 'RMC' and msg.status == 'A':         # 데이터 형식이 RMC임을 확인, 유효한 값인지 확인
                print("%f, %f" % (msg.latitude, msg.longitude))

                # 원본 위경도 데이터 저장
                fw.write("%f, %f" % (msg.latitude, msg.longitude) + '\n')

        if keyboard.is_pressed('Esc'):
            fw.close()
            print('save and fin')
            break


    except serial.SerialException as e:
        print('Device error: {}'.format(e))
        pass

    except pynmea2.ParseError as e:
        print('Parse error: {}'.format(e))
        pass

fw.close()

print('fin')