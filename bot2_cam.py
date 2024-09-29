import sensor, image, time, pyb, math, array
from pyb import UART

angle_list = [] #get possible angles for one quadrant given x, y

for x in range(121):
    col = []
    for y in range(121):
        angle = math.atan2(x, y) * 180 / math.pi
        angle = math.fmod((angle / 360) * 240, 240.0)
        col.append(angle)
    angle_list.append(col)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_windowing((48, 0, 240, 240))
sensor.set_gainceiling(128)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False) # must be turned off for color tracking

sensor.skip_frames(time=500)
sensor.set_auto_exposure(False, exposure_us=12000)
sensor.set_auto_gain(False, gain_db = 4)
sensor.set_auto_whitebal(False)

#print("gain", sensor.get_gain_db())
#print("exp",sensor.get_exposure_us())

uart = UART(3, 500000, timeout_char=1000)
sensor.skip_frames(time=500)
clock = time.clock()
green_led = pyb.LED(2)

bot_x = 120
bot_y = 120
max_x = 240
max_y = 240

mask = image.Image('/test2.pgm', copy_to_fb=False)
mask.to_bitmap()

ball_thresh = (39, 100, 1, 47, 23, 98)#(37, 85, -3, 105, 10, 127) #(31, 85, -2, 106, 25, 72)(44, 78, 19, 92, 14, 71)
ygoal_thresh = (70, 89, -26, 8, 22, 99) #(34, 87, -44, 7, 22, 79)
bgoal_thresh = (46, 85, -29, -10, -35, -17)#(47, 64, -35, -8, -49, -14) #(45, 68, -27, -8, -34, -17) #(48, 63, -26, -11, -128, -20)

end_char = 254

front_goal = 1 #blue in front
use_goal = 1 #use blue goal

goal_right_angle = 255
goal_left_angle = 255
goal_real_dist = 255
bot_cam_x = 255
bot_cam_y = 255
goal_right_tan = 255
goal_left_tan = 255
goal_right_constant = 255
goal_left_constant = 255

while (True):
    clock.tick()
    img = sensor.snapshot()#.clear(mask)
    #img.draw_circle(120, 120, 10)
    #img.draw_cross(120, 120)

    ball_blobs = img.find_blobs([ball_thresh], pixel_threshold=0, area_threshold=0, merge=True)
    if (use_goal): goal_blobs = img.find_blobs([bgoal_thresh], pixel_threshold=100, area_threshold=50, merge=False)
    else: goal_blobs = img.find_blobs([ygoal_thresh], pixel_threshold=100, area_threshold=50, merge=False)

    #BALL STUFF
    if not len(ball_blobs):
        #green_led.off()
        ball_angle = 255
        ball_pix_dist = 255
    else:
        #green_led.on()
        x = max(ball_blobs, key = lambda x:x.pixels())
        #img.draw_rectangle(x.rect())

        x_coord = bot_x - x.cx() #top right  in frame is actually top left  irl so flipped the x  to b increasing from r to l in frame (left to right irl)
        y_coord = (max_y - x.cy()) - bot_y

        ball_angle = angle_list[abs(x_coord)][abs(y_coord)]
        if x_coord > 0:
            if y_coord > 0:
                ball_angle = ball_angle
            else:
                ball_angle = 120 - ball_angle
        else:
            if y_coord > 0:
                ball_angle = 240 - ball_angle
            else:
                ball_angle = 120 + ball_angle

        ball_pix_dist = math.floor((x_coord ** 2 + y_coord ** 2) ** 0.5)
        ball_pix_dist = min(253, max(0, ball_pix_dist)) #sld b max 170 ish

    #GOAL STUFF
    goal_right_x, goal_right_y, goal_left_x, goal_left_y = max_x, 0, 0, 0

    if not len(goal_blobs):
        goal_right_angle = 255
        goal_left_angle = 255
        goal_real_dist = 255
        bot_cam_x = 255
        bot_cam_y = 255
        goal_right_tan = 255
        goal_left_tan = 255
        goal_right_constant = 255
        goal_left_constant = 255
    else:
        for x in goal_blobs:
            #img.draw_rectangle(x.rect())
            corners = list(x.corners())

            corners.sort(key = lambda x:x[0])

            top_left = corners[3]
            top_right = corners[0]

            #img.draw_cross(top_left)
            #img.draw_cross(top_right)

            #get angles of left/rightmost corners
            if top_right[0] < goal_right_x:
                goal_right_x = top_right[0]
                goal_right_y = top_right[1]

                goal_reframed_right_x = bot_x - goal_right_x
                goal_reframed_right_y = (max_y - goal_right_y) - bot_y
                goal_right_angle = angle_list[abs(goal_reframed_right_x)][abs(goal_reframed_right_y)]

                if goal_reframed_right_x > 0:
                    if goal_reframed_right_y > 0:
                        goal_right_angle = goal_right_angle
                    else:
                        goal_right_angle = 120 - goal_right_angle
                else:
                    if goal_reframed_right_y > 0:
                        goal_right_angle = 240 - goal_right_angle
                    else:
                        goal_right_angle = 120 + goal_right_angle

            if top_left[0] > goal_left_x:
                goal_left_x = top_left[0]
                goal_left_y = top_left[1]

                goal_reframed_left_x = bot_x - goal_left_x
                goal_reframed_left_y = (max_y - goal_left_y) - bot_y
                goal_left_angle = angle_list[abs(goal_reframed_left_x)][abs(goal_reframed_left_y)]

                if goal_reframed_left_x > 0:
                    if goal_reframed_left_y > 0:
                        goal_left_angle = goal_left_angle
                    else:
                        goal_left_angle = 120 - goal_left_angle
                else:
                    if goal_reframed_left_y > 0:
                        goal_left_angle = 240 - goal_left_angle
                    else:
                        goal_left_angle = 120 + goal_left_angle

            if ((goal_right_angle < 60 or goal_right_angle > 180) and (goal_left_angle < 60 or goal_left_angle > 180)): #is goal in front
                front_goal = 1
            else:
                front_goal = 0

        calc_goal_left_angle = goal_left_angle / 240 * 360
        calc_goal_right_angle = goal_right_angle / 240 * 360
        # reject if error to 90 is < certain value
        # min should be around 4-5 at min point
        left_err_from_90 = 90 - abs(math.fmod(calc_goal_left_angle + 270, 180) - 90)
        right_err_from_90 = 90 - abs(math.fmod(calc_goal_right_angle + 270, 180) - 90)
        #print(left_err_from_90, right_err_from_90)
        if min(left_err_from_90, right_err_from_90) < 6:
            # reject
            goal_right_angle = 255
            goal_left_angle = 255
            goal_real_dist = 255
            bot_cam_x = 255
            bot_cam_y = 255
            goal_right_tan = 255
            goal_left_tan = 255
            goal_right_constant = 255
            goal_left_constant = 255
        if (front_goal):
            calc_goal_left_angle = (360 + 90 - calc_goal_left_angle) % 360
            calc_goal_right_angle = (360 + 90 - calc_goal_right_angle) % 360

            goal_left_tan = math.tan(calc_goal_left_angle * math.pi / 180)
            goal_right_tan = math.tan(calc_goal_right_angle * math.pi / 180)

            goal_left_constant = 2160 - goal_left_tan * 610
            goal_right_constant = 2160 - goal_right_tan * 1210
        else:
            calc_goal_left_angle = 270 - calc_goal_left_angle
            calc_goal_right_angle = 270 - calc_goal_right_angle

            goal_left_tan = math.tan(calc_goal_left_angle * math.pi / 180)
            goal_right_tan = math.tan(calc_goal_right_angle * math.pi / 180)

            goal_left_constant = 270 - goal_left_tan * 610
            goal_right_constant = 270 - goal_right_tan * 1210

        try:
            bot_cam_x = (goal_left_constant - goal_right_constant) / (goal_right_tan - goal_left_tan) #(c2-c1)/(m1-m2)
            bot_cam_y = goal_right_tan * bot_cam_x + goal_right_constant
        except ZeroDivisionError:
            # two angles are the same (extreme case) -- cannot calculate angle
            # but dont reject cos can still be used for bounds for x
            pass
        else:
            bot_cam_x /= 10 #convert to cm
            bot_cam_y /= 10

            # calculate distance from back centre of goal
            if (front_goal): goal_real_dist = min(((61 - bot_cam_x) ** 2 + (223.4 - bot_cam_y) ** 2) ** 0.5, ((121 - bot_cam_x) ** 2 + (223.4 - bot_cam_y) ** 2) ** 0.5)
            else: goal_real_dist = min(((61 - bot_cam_x) ** 2 + (19.6 - bot_cam_y) ** 2) ** 0.5, ((121 - bot_cam_x) ** 2 + (19.6 - bot_cam_y) ** 2) ** 0.5)
            #print(goal_left_angle/240*360, goal_right_angle/240*360, goal_real_dist)

            #middle angle (not used rn)
            '''
            goal_ang_width = (goal_right_angle - goal_left_angle + 120) - math.floor((goal_right_angle - goal_left_angle) / 240) * 240 - 120
            if goal_ang_width > 120: goal_ang_width = 240 - goal_ang_width

            if (front_goal): goal_mid_angle = (goal_left_angle + (0.5 * goal_ang_width)) % 240
            else: goal_mid_angle = (goal_right_angle + (0.5 * goal_ang_width)) % 240
            '''

    #print(f"using: {use_goal} front: {front_goal} left: {goal_left_angle/240*360} right: {goal_right_angle/240*360} x: {bot_cam_x} y: {bot_cam_y} real: {goal_real_dist}")

    #switch goals?
    if (use_goal and goal_real_dist > 85): use_goal = 0
    elif (not use_goal and goal_real_dist > 85): use_goal = 1

    #print(f"goal: {front_goal} left: {goal_left_angle/240*360} right: {goal_right_angle/240*360}")
    #print(f"usegoal: {use_goal} goal: {front_goal} left: {goal_left_angle/240*360} right: {goal_right_angle/240*360} box: {bot_cam_x} boy: {bot_cam_y}")

    uart.writechar(math.floor(ball_angle))
    uart.writechar(ball_pix_dist)
    uart.writechar(math.floor(goal_left_angle))
    uart.writechar(math.floor(goal_right_angle))
    #uart.writechar(math.floor(bot_cam_x)) #decided not to send down so can correct for gy
    #uart.writechar(math.floor(bot_cam_y))
    uart.writechar(end_char)

    #print(clock.fps())
