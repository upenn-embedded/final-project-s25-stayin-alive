---
layout: default
---

# Baby CPR Project
Created by Shruti Agarwal, Howard Xu, Zora Mardjoko

We are excited to present our functional baby CPR training device that automates chest compressions using a motorized crank-slider mechanism, with adjustable compression speed and synchronized audio instructions. 
Watch the demo below:

{% include youtube.html id="kBenaHzO5P8" %}

## Images
![alt text](IMG_8390.jpg)
![alt text](image.png)


## 3. Results

Our final solution is a functional baby CPR training device that automates chest compressions using a motorized crank-slider mechanism, with adjustable compression speed and synchronized audio instructions. We implemented PWM-based motor control to vary compressions between 100–120 BPM, adjusted via a slider. Compression frequency is displayed on an LCD screen. An encoder tracks compression frequency, while an external soundboard provides real-time verbal guidance. We also implemented a start/stop button for compressions.
### 3.1 Software Requirements Specification (SRS) Results

We successfully implemented motor-based CPR compressions with dynamic BPM control and audio-guided instructions. The firmware handles PWM motor driving, ADC-based speed control, and audio playback. We were not able to get the heart rate sensor working, which was one of our requirements. However, we integrated an LCD screen, utilizing the LCD graphics library provided in class to fulfill the SPI communication requirement.

| ID     | Description                                                                                                                                           | Validation Outcome                                                                                                                                                                                                 |
|--------|-------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SRS-1  | Timing 100–120 compressions per minute via PWM                                                                                                       | **Achieved**. Duty cycles between 40–50% achieved desired BPM. Oscilloscope and encoder frequency confirm accurate timing. Serial output verifies BPM computation (see [video](https://drive.google.com/file/d/18KjJh7z2JQtU2WsWvnEp5noG2TvVmWYm/view)). |
| SRS-2  | Input capture to time stopping compressions around 15 seconds into the sequence                                                                      | **Not achieved**. Requirement was dropped in favor of a continuous compression sequence.                                                                                                                           |
| SRS-3  | ADC input to modulate compression frequency                                                                                                           | **Achieved**. Measured ADC from slider. See [demo video](https://youtube.com/shorts/kBenaHzO5P8).                                      |
| SRS-4  | Voice instructions synchronized with motor operation                                                                                                  | **Achieved**. Audio cues from a soundboard are synchronized with compression behavior. [Video 1](https://drive.google.com/file/d/1d2OzKgbDKBN98MRBmpH-0eDDmVQTMjfH/view) [Video 2](https://drive.google.com/file/d/1HsucxccEQHHMFxV_PzVdPIGKXBBRkYkZ/view) |
| SRS-5  | Start/stop button using pin change interrupts                                                                                                         | **Achieved**. Button works to start and stop compressions. See [demo video](https://youtube.com/shorts/kBenaHzO5P8).                                                                                                                                                     |
| SRS-6  | Polling or interrupting heart rate sensor for stable BPM                                                                                              | **Not achieved**. We pivoted to implementing an LCD screen for the project SPI communication.                                                                                                                        |

---

### 3.2 Hardware Requirements Specification (HRS) Results

Our hardware implementation delivered reliable compression actuation, sound output, and ADC-based control. As mentioned in the SRS results, we were not able to integrate the heart rate sensor into our system, opting instead for an LCD screen.

| ID     | Description                                                                                                                                      | Validation Outcome                                                                                                                                                                                                                      |
|--------|--------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| HRS-1  | Crank mechanism shall produce 1.5 inches of motion                                                                                               | **Adjusted**. Due to the small doll size, we reduced depth to 0.75 inches. Validated via CAD screenshot. <br> ![crank-depth](https://github.com/user-attachments/assets/fd779a98-1159-4f63-96bc-62103fc9ed09)                          |
| HRS-2  | Speaker shall be audible in noisy environments (>70 dB)                                                                                         | **Achieved**. Decibel readings confirm speaker output exceeds 70 dB. Audio samples in [Video 1](https://drive.google.com/file/d/1d2OzKgbDKBN98MRBmpH-0eDDmVQTMjfH/view). <br> ![Speaker reading](https://github.com/user-attachments/assets/2311e280-9195-4949-bc87-fab6b7d8cf40)                                  |
| HRS-3  | Slider shall vary compression speed via ADC                                                                                                      | **Achieved**. Functionally validated through [demo video](https://youtube.com/shorts/kBenaHzO5P8).                                                                                                                                                                   |
| HRS-4  | Heart rate sensor shall detect BPM within ±5 BPM                                                                                                 | **Not achieved**. Sensor integration was scrapped due to driver/library challenges. Pivoted to LCD screen for SPI communication.                                                                                                                 |
| HRS-5  | Compression cadence shall be within ±10% of desired 100–120 BPM                                                                                  | **Achieved**. Captured range is 102–121 BPM as shown in [video](https://drive.google.com/file/d/18KjJh7z2JQtU2WsWvnEp5noG2TvVmWYm/view) and [demo video](https://youtube.com/shorts/kBenaHzO5P8).                                                                                                                                                       |
| HRS-6  | Mechanism shall withstand 100+ compressions                                                                                                       | **Achieved**. Mechanically tested and verified over 100 compressions with no degradation.                                                                                                                                               |


## Conclusion
Through this project, we learned how to integrate hardware and software to create a functional baby CPR training device. We’re proud that we were able to meet most of our hardware and software requirements, particularly in implementing SPI communication using an LCD screen after encountering unexpected issues with the heart rate sensor. Although the sensor didn’t work as planned, adapting our approach and finding a workaround was a valuable learning experience. Integrating the LCD onto the ATMega was challenging due to its pin usage, requiring us to rewire and reimplement other parts of the system. We also faced delays with 3D printing due to limited lab access, which impacted our timeline slightly. If we were to do another similar project, we would ensure that 3D printing was started as soon as possible to account for delays/mistakes in printing jobs. A potential next step would be fully integrating a working heart rate sensor to enhance the device’s realism and functionality.

## References
For the LCD screen, we used the graphics library provided in class for the Pong lab.

Additionally, we referenced lots of documentation for the heart rate sensor, which we didn't end up implementing. The drivers/documentation can be found below: 
C driver: https://github.com/libdriver/max30102/blob/main/src/driver_max30102.c

Arduino Cpp driver: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/tree/master
