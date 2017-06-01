# test_serial
Quick 20 minute prototype of serial communication with arduino from RPi using python-periphery.


```shell
$ pip install pyserial
$ cd pio_proj
$ pio run -t upload
$ cd ..
$ python arduino_handler.py
```

Message Pi -> Arduino:

```CSV
status,                     // 0
pump_1_nutrient_a_1,        // float flow_rate
pump_2_nutrient_b_1,        // float flow_rate
pump_3_ph_up_1,             // bool on_off (pulse limited)
pump_4_ph_down_1,           // bool on_off (pulse limited)
pump_5_water_1,             // bool on_off (immediate)
chiller_fan_1,              // bool on_off (immediate)
chiller_pump_1,             // bool on_off (immediate)
heater_core_2_1,            // bool on_off (immediate)
air_flush_1,                // float on_minutes
water_aeration_pump_1,      // bool on_off (immediate)
water_circulation_pump_1,   // bool on_off (immediate)
chamber_fan_1,              // bool on_off (immediate)
led_blue_1,                 // float 0-1 pwm
led_white_1,                // float 0-1 pwm
led_red_1,                  // float 0-1 pwm
heater_core_1_1,            // bool on_off (immediate)
chiller_compressor_1\n      // bool on_off (immediate tone)
```

Message Arduino -> Pi:

OK:
```csv
0, // status: OK
air_humidity,
air_temperature,
air_carbon_dioxide,
water_temperature,
water_level_low,
water_level_high,
water_potential_hydrogen,
water_electrical_conductivity\n
```

WARN/ERROR:
```CSV
status, // 1: WARN, 2: ERROR
DeviceName,
status_code,
status_msg\n
```
