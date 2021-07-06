# TDPS-team-design-project-for-Glasgow-college-of-UESTC
team46：Fast&Furious

大三的设计项目，和团队的小伙伴们一起完成的小车，能分别独立完成两个patio。在此表达对队友大佬的感谢，以及对英方老师祖宗的问候（

这里只会分享本人负责patio1部分及代码，希望对之后的小伙伴有帮助


configuration:

1. 使用cube IDE 平台完成小车驱动，任务实现，中断等功能
2. 使用openMV实现边缘检测，颜色识别
3. 使用HC-SR04以及US-100超声波模块检测距离（告诫各位，一分价钱一分货,虽然两个都很容易抽风）
4. 小车别选太大的车架，否则上桥和过拱门很吃亏
5. 我们使用的是stm32f446开发板，以及openMV4 h7图像处理模块

6. 建议在选定stm32型号后多买几块stm32和openmv，能同时和队友一起开发，最重要的是有个备用，很重要！！！
   有很多组都是临测试前几天把板子和openMV烧了，临时买是来不及的
7. 上一条适用于所有元件


results and discussion for patio1:
Task1:
In task1, line patrol, the related functions are line_tracking(void), straight(s_angle) and turning(s_angle), where line_tracking and straight are developed as callback functions of interrupt with Cube IDE. The achievement of line patrol will be triggered by a switch attached to the microcontroller stm32f446. When switched on, the GPIO pin 13 is set, and the interrupt with timer2 will be open by 

HAL_TIM_Base_Start_IT(&htim2);

Inside the callback function, the microcontroller will begin to receive data transmitted from OpenMV, which is an integer i. When i equals to 0, it means OpenMV has not detected any boundaries and the rover will move straight. When i equals to 1, it means OpenMV detected boundary on the right, and the rover is supposed to adjust its direction to left. When i equals to 2, it means OpenMV detected boundaries on the left side and the rover is supposed to adjust its direction to right. OpenMV will keep detecting boundaries with a rather high frequency, which makes it possible to adjust direction of motion before the rover biasing the trace. 

Inside the line_tracking() function, the speed of two motors are controlled by 2 PWM waves generated by microcontroller. Meanwhile, the speed is adjustable by applying PWM waves with different duty cycle after a constant time, the line_tracking interrupt will be stopped with 

HAL_TIM_Base_Start_IT(&htim2);
and task1 is finished. 
![image](https://user-images.githubusercontent.com/86994632/124536624-db1d2a80-de4a-11eb-8cf1-76dd1c7d2e72.png)
 

Task2:
In task2, the related functions are get_distance(void), turning(angle) and straight(void), where get_distance and straight functions are developd as interrupt. when line_trcking() interrupt is closed, another interrupt with uart6 will be triggered in 2 seconds, which will determine the front distance of the rover. The corresponding callback function is get_distance(), meanwhile the interrupt will start with statements: 

HAL_UART_Transmit_IT(&huart6,&sign,1);
HAL_UART_Receive_IT(&huart6,receive,2);
HAL_Delay(50);

 As rover gets close to the beacon, the returned value of distance will decrease. When the returned value is smaller than the predefined constant given_dis0, the main program will call the turning function to adjust the direction for getting on the bridge. The turning angle is determined by gyroscope in advance, and will be passed to the turning () function. When the current position angle of rover matches with the given s_angle, the main program will stop the loop body and the rover will stop turning. Then, the duty cycle of two motors are reset to provide enough power to get on the bridge:
 
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 62);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 60);
HAL_Delay(5000);

As long as the rover stop turning, the ultrasonic ranging module will be disabled by stopping the corresponding interrupt with uart6. On the bridge, the interrupt with timer3 will be triggered, and its callback function is straight function. With assistance od gyroscope, the straight function can adjust the duty cycle of motors to ensure the rover moves along the same angle defined in advance, which is identical to the direction of bridge. 
 
s_angle=straight_angle2;
HAL_TIM_Base_Start_IT(&htim3);

![image](https://user-images.githubusercontent.com/86994632/124536663-ee2ffa80-de4a-11eb-9794-6d85b50f4295.png)


Task3:
After crossing the bridge, the interrupt with uart6 will be triggered again to call the distance detection function, and the front distance will be returned to the main program as criterion of when to switch to task3. As raging module works, the parallel interrupt callback function straight () works as well to adjust moving position of rover, which ensures the second beacon will be detected successfully. 

As long as the front distance meets the set constant given_dis1, the function turning () will be called to force rover to turn to the direction of arc. When the error of direction is within the interval +1/-1, the function will break from loop body of turning () and enable the interrupt with timer2. Then, rover will move towards the arc with assistance of interrupt callback function line_tracking(), and stop by calling function stop() after crossing the arc in a few seconds.

In task3, the duration for line patrol is set to be 10 seconds, which is adequate for rover to successfully cross the arc. After interrupt related to line patrol is stopped, the rover requires to stop by calling stop () function. In the stop () function, the duty cycle of both motors will be set to 0, and rover will stop immediately. This is the end of task3 and patio1.
![image](https://user-images.githubusercontent.com/86994632/124536692-fb4ce980-de4a-11eb-9de1-2fbc6d033d38.png)

 


Discussion
Task1:
In the real test of task1, several problems occurred in sequence. At the beginning, OpenMV did not execute the boundary detection program precisely due to impact of light intensity. To solve the problem, we had to adjust the detecting threshold value before each test in real spot, which ensures OpenMV worked well enough to handle the boundaries of road.

Then we found that the ground is much rougher compared with floor of classroom. In this case, the rover requires higher power lever of motors to maintain the basic pace of line patrol and turning precisely. Therefore, the duty cycle of motors is increased to 40%, which is acceptable for boundary detection with OpenMV in a given frequency. 
After 2 weeks test, the line patrol task was achieved successfully without external disturbance, and the duration of task1 was decreased to 3 minutes. 


Task2:
In the real test of task2, the criterion for beginning of task1 is the front distance to first beacon. After line patrol, the interrupt of distance detection with ultrasonic ranging module is triggered to determine the front distance in a set frequency. However, it sometimes happened that the interrupt was not triggered as we expected and the returned value is constant. Initially the problem was considered to be related with priorities of interrupts. In order to solve the problem, we activate the FreeRTOS function of Cube IDE to manage used interrupts. But the situation was not improved. Finally, in order to ensure ultrasonic module works functionally, we added a testing program at the beginning of the patio1. If error occurs, we will be informed by remote communication and reset the program before starting the whole patio. 

Meanwhile, we found that each time the rover got over the bridge, the direction of rover was undetermined. We checked the condition of rover in motion, and it shown that the asymmetry of caterpillar bands is the reason. Because the relative location of caterpillar bands are fixed, we adopted the straight () function to adjust direction of rover after crossing the bridge. With assistance of straight () function, the rover can move with a given angle and adjust automatically if biased. 

After 1 week test, the bridge-crossing task was achieved successfully, but artificial adjustments were required when direction of rover was biased. The duration of task2 was limited to 1 minutes.


Task3:
In the initial design of task3, the criterion of whether rover had crossed the arc is related to side ultrasonic ranging module. However, due to the limitation of detecting frequency, the ultrasonic module was unable to detect the arc always successfully, even the rover moved in a rather low speed. In this case, we chose to set a duration time of task3 in order to make the rover stop after crossing the arc. the starting point of timer is the moment rover met with the second beacon. 

After finishing task2, the rover enables front ultrasonic ranging module again to detect the second beacon. When the second beacon is detected, rover began to turning to the direction of arc and initialized the timer. The rover will move to the arc and stop in a few seconds after crossing the arc. The set time is approximately 10 seconds.
After 1 week test, the arc-crossing task was achieved successfully, in which line_tracking function still played an important role. The duration of task3 was limited to 1 minutes. 





personal contribution:

main program for patio1:
From April to May, I was appointed to work on the main program of patio1. In April, I derive the initial approach to achieve tasks in patio1, including the basic flow chart and pseudo code. Later, I code the main program of patio1 based on the Mbed platform, in which the operation of rovers is fulfilled by calling predefined functions. However, Mbed is not visible to configure the pin and clocks compared with Cube IDE, so I transplanted code to Cube IDE and configure timers and connectivity visibly on it. Then with assistance of teammates, I finished the code of main program of patio1 with interrupts and packaged call-back functions. 

The code of main program is constructed with 3 tasks, line patrol, crossing bridge and going through gate. In task1, functions called in main program are line_tracking(void), straight(void) and turning(i), where line_tracking function is developed as an interrupt on Cube IDE. In task2, functions called in the main program are get_distance(void), turning(angle) and straight(void), where get_distance and straight functions are developd as interrupt. In task3, functions called in main program are stop(void), line_tracking(void) and get_distance(void). In task3, the time of duration before stop is determined by timer. 

In task1, after switching on, the interrupt at pin13 is triggered, in which the duty cycle for each motor will be assigned. Then predefined timer2 is enabled, while interrupt of line tracking with timer2 is triggered simultaneously. Inside interrupt callback function of timer2, the line_tracking() function is called. After finishing line patrol task, the interrupt with pin function timer2 will be stopped before detecting front distance. 

![image](https://user-images.githubusercontent.com/86994632/124536591-d22c5900-de4a-11eb-9fa3-d1f8cdb78b87.png)
![image](https://user-images.githubusercontent.com/86994632/124536624-db1d2a80-de4a-11eb-8cf1-76dd1c7d2e72.png)

In task2, before the rover is in position to move on the bridge, the front ultrasonic ranging module begins to work in the interrupt callback function get_distance() with pin function uart6. When the measured distance returned to main program is less than given_dis0, which is a preannounced constant, the ultrasonic ranging module will be disabled and the turning function turning (float s_angle) will be called. After turning to the direction of bridge, the loop inside turning (float s_angle) ends and the rover begins to go straight with constant speed.

![image](https://user-images.githubusercontent.com/86994632/124536660-eb350a00-de4a-11eb-9a0d-47aedbabedea.png)
![image](https://user-images.githubusercontent.com/86994632/124536663-ee2ffa80-de4a-11eb-9794-6d85b50f4295.png)

In task3, after crossing the bridge, the interrupt with uart6 is enabled again to determine the front distance. When the measured distance returned to main program is less than given_dis1, which is a preannounced constant, the ultrasonic ranging module will be disabled and the turning function turning (float s_angle) will be called again. This time, the direction is towards the arc. When the direction is correct, the interrupt related to line patrol will be triggered, and the rover will stop in a period of time after crossing the arc.

![image](https://user-images.githubusercontent.com/86994632/124536688-f8ea8f80-de4a-11eb-8a96-9c2b98016d3d.png)
![image](https://user-images.githubusercontent.com/86994632/124536692-fb4ce980-de4a-11eb-9de1-2fbc6d033d38.png)

From late May to the end of the project, I attended every test in the spot of patio1. Compared with simulation with computer, there exists a number of influence factors in the real situation, such as light intensity, temperature, limit capacity and voltage of batteries. In order to accelerate the program under test, I broke the main program into pieces and decreased the frequency of boundary detection. Meanwhile, I added several judging statements before the execution of each task, which enhanced the robustness of main program. 
Some problems occur in real spot. For instance, in task 2, the bridge floor is constructed with steel fence, which is unfriendly to our crawler-type rover, because our straight function will be affected by the condition of bridge. In order to solve the problem, we had to adjust the duty cycle of each motor to ensure the rover moving straight on the bridge.










