Project info
=======

This project contains 2 boards with different sensor sets.

Board#1:
- CO2
- TVOC
- NOx
- Temperature
- Humidity
- PMx (dust)
- H2S (hydrogen sulfide)
- CO (carbon monoxide)
- O2 (oxygen)

Board#2
- TVOC
- NOx
- Temperature
- Humidity
- H2S (hydrogen sulfide)
- Light (in percents, not in Lumens)

It controls:
- Exhaust fan 
- Cooling fan with speed control (fan_pwm driver) 

Additional features:
- Integrates with any smart-house system via MQTT protocol (over WIFI)
- Contains touchpad supports multi-click (single click, double click, tripple click etc)
- Contains RGB led to indicate some internal state you want
- OTA updates - just publish new firmware on your local HTTP server and send URL to it into MQTT topic

3D models
=======
- Board#1 - 3d/box-slepingroom*.stl
- Board#2 - 3d/box-bathroom*.stl

Home assistant files
=======
- Board#1 - in progress
- Board#2 - hass/board2

Project status
=======

- Board#1 - Done
- Board#2 - Done

Photos
=======
Board#1:
![photo_2024-05-03_17-02-17](https://github.com/shm-dmitry/air-detector/assets/19342331/c1340e0f-9584-4514-bf00-15b94fbbcf35)
![photo_2024-05-03_17-02-14](https://github.com/shm-dmitry/air-detector/assets/19342331/8c005576-3a74-4646-82e2-904d149de9e6)
![photo_2024-05-03_17-02-11](https://github.com/shm-dmitry/air-detector/assets/19342331/d2a3e5e8-70a9-4216-9daf-9c24864fbee2)
![photo_2024-05-03_17-02-09](https://github.com/shm-dmitry/air-detector/assets/19342331/5cc4f24e-31e9-44f0-9580-9caf0c2112fa)
![photo_2024-05-03_17-02-06](https://github.com/shm-dmitry/air-detector/assets/19342331/60dd59fe-d786-42a7-b85e-4557230d4438)


Board#2:
![изображение](https://github.com/shm-dmitry/air-detector/assets/19342331/f33c22ae-c708-4396-b865-effaadd7a6bc)
![изображение](https://github.com/shm-dmitry/air-detector/assets/19342331/f1fc3584-b8d8-4422-8663-e24f16bf7604)
![изображение](https://github.com/shm-dmitry/air-detector/assets/19342331/38499ebe-4c6b-4ac5-b464-c3dfeb263745)
![изображение](https://github.com/shm-dmitry/air-detector/assets/19342331/3bfaf705-65f7-4504-8d73-c2d4607b93e4)


