# VibrationRecorder

for STM32F765

### After using CubeMx to re-generate code:
- SD card

subsititude sd_diskio.c/.h (v2.1->v1.4)

- RTC

```
sDate.Year      = 0x18;
sDate.Month     = 0x04;
sDate.Date      = 0x24;
sTime.Hours     = 0x13;
sTime.Minutes   = 0x23;
sTime.Seconds   = 0x18;
```