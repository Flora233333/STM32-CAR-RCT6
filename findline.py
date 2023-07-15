import sensor
import image
import lcd
import time
from pyb import UART, LED
import struct

green_led = LED(2)
green_led.toggle()

lineblods_threshold   = (0, 15, -7, 3, -5, 9)#(6, 52, 7, 61, -1, 45)#(0, 20, -45, 44, -30, 13)

ROIS = [
        (10, 30, 300, 50),
        (10, 100, 300, 50),
        (10, 170, 300, 50)
       ]#三个区域


averge_x = 0
left_averge_x = 0
right_averge_x = 0
cnt_x = 0
num_of_blobs = []
flag_of_3cross = [0, 0]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(n=20)
# sensor.set_windowing((224, 224)) # 这里可以改，因为这是也要用神经网络所以resize了
sensor.set_hmirror(False)
sensor.set_vflip(False)


uart = UART(3, 115200)


def send_result(left, right):
    #print(buf)
    ## data = [0x7B]
    #data = struct.pack("<bbb",              #格式为3个字符
                #0xFF,                       #帧头1
                #int(buf / 2),
                #0xFE,
                #)
    ## data.append(int(buf))
    ## data.append(0x6D)
    ## print(data)
    #uart.write(data)
    data = [0xFF]
    data.append(int(left / 2 + 0.5))
    data.append(int(right / 2 + 0.5))
    data.append(0xFE)
    print(data)
    uart.write(bytearray(data))
    # time.sleep_ms(1)


def judge_bypass_3cross(): # 判断是否经过一次三叉
    global num_of_blobs, flag_of_3cross

    if (2 in num_of_blobs or 3 in num_of_blobs) and len(num_of_blobs) == 3:

        if (flag_of_3cross[0] == 0 and num_of_blobs[0] == 2
                                   and num_of_blobs[1] == 2
                                   and num_of_blobs[2] == 1): # 可以去掉最后一个，因为有点难看到
            flag_of_3cross[0] = 1 # flag_of_3cross[0] 是加锁的flag，相当于检测到一个ROI中有两个方框后，加锁，再三个ROI都变成一个方框后解锁
            #print('3cross')
        if (flag_of_3cross[0] == 0 and num_of_blobs[0] == 1
                                   and num_of_blobs[1] == 2
                                   and num_of_blobs[2] == 1):
            flag_of_3cross[0] = 1

        if (flag_of_3cross[0] == 0 and num_of_blobs[0] == 1
                                   and num_of_blobs[1] == 2
                                   and num_of_blobs[2] == 2):
            flag_of_3cross[0] = 1

    elif (len(num_of_blobs) == 3 and num_of_blobs[0] == 1
                                 and num_of_blobs[1] == 1
                                 and num_of_blobs[2] == 1):

        if flag_of_3cross[0] == 1:
            flag_of_3cross[0] = 0
            flag_of_3cross[1] += 1 # 表明经过一次三叉路口
        #print('strght', flag_of_3cross[1])

    num_of_blobs = []




def find_max_lines_blods(img, roi):
    global averge_x, cnt_x, num_of_blobs, flag_of_3cross, left_averge_x, right_averge_x

    max_Pixels = [0, 0]
    max_ID = [0, 0]

    if img:
        blobs = img.find_blobs([lineblods_threshold], roi=roi, area_threshold=500, pixel_threshold=500,merge=True, margin=3)
        if blobs:

            num_of_blobs.append(len(blobs))

            for n in range(len(blobs)):
                if blobs[n].pixels() > max_Pixels[0]:
                    max_Pixels[1] = max_Pixels[0]
                    max_ID[1] = max_ID[0]
                    max_Pixels[0] = blobs[n].pixels()
                    max_ID[0] = n
                elif blobs[n].pixels() > max_Pixels[1]:
                    max_Pixels[1] = blobs[n].pixels()
                    max_ID[1] = n

            if len(blobs) >= 2: # blobs >= 2 时区分左右
                # print(2)
                if blobs[max_ID[0]].cx() > blobs[max_ID[1]].cx():
                    left_x = blobs[max_ID[1]]
                    right_x = blobs[max_ID[0]]
                else:
                    left_x = blobs[max_ID[0]]
                    right_x = blobs[max_ID[1]]


                img.draw_rectangle(right_x.rect())
                img.draw_rectangle(left_x.rect())

                img.draw_cross(right_x.cx(), right_x.cy())
                img.draw_cross(left_x.cx(), left_x.cy())

                # if flag_of_3cross[1] == 2 or flag_of_3cross[1] == 3: # 已经过三叉路口的次数
                #     averge_x += left_x.cx() # 走内圈
                #     cnt_x += 1
                # else:
                #     averge_x += right_x.cx() # 走外圈
                #     cnt_x += 1
                left_averge_x += left_x.cx()
                right_averge_x += right_x.cx()

                cnt_x += 1

            elif len(blobs) == 1:
                # print(1)
                img.draw_rectangle(blobs[max_ID[0]].rect())
                img.draw_cross(blobs[max_ID[0]].cx(), blobs[max_ID[0]].cy())

                left_averge_x += blobs[max_ID[0]].cx()
                right_averge_x += blobs[max_ID[0]].cx()
                # averge_x += blobs[max_ID[0]].cx()
                cnt_x += 1

        else:
            return 0
    else:
        return 0

while True:
    img = sensor.snapshot()


    #for j in ROIS:
        #find_max_lines_blods(img, j)

    #if averge_x != 0:
        #averge_x /= cnt_x
        ## print(averge_x, cnt_x)
        #send_result(int(averge_x))
        #averge_x = 0
        #cnt_x = 0


    #histogram = img.get_hstogram()
    #Thresholds = histogram.get_threshold()
    #img.binary([(Thresholds.value(), 255)])

    #img.binary([(0,50)], invert=True)
    #img = img.mean(1)

    for j in ROIS:
        find_max_lines_blods(img, j)

    #judge_bypass_3cross()
    #print(flag_of_3cross[1])

    # if averge_x != 0:
    #     averge_x /= cnt_x
    #     print(int(averge_x), cnt_x, flag_of_3cross[1])
    #     send_result(int(averge_x))
    #     averge_x = 0
    #     cnt_x = 0

    if left_averge_x != 0 and right_averge_x != 0:
        left_averge_x /= cnt_x
        right_averge_x /= cnt_x
        print(int(left_averge_x), int(right_averge_x), cnt_x)
        send_result(left_averge_x, right_averge_x)
        left_averge_x = 0
        right_averge_x = 0
        cnt_x = 0

    # lcd.display(img)
