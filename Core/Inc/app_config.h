extern uint32_t interval_ds18 ; // миллисекунды, min 800                        [A800]
extern uint32_t interval_ec; // миллисекунды, min 500                           [B

extern uint32_t referenceVoltage; // 2500mv (напряжение питания датчиков)       [C

extern uint32_t ntcR1; // 10kΩ voltage divider resistor value                   [D
extern uint32_t ntcRo; // 10kΩ R of Thermistor at 25 degree                     [E
extern uint32_t ntcTo; // 25 Temperature in Kelvin for 25 degree                [F
extern uint32_t ntcKoefB; // 3950 Beta value                                    [G

extern int32_t ecRo; //  Ω Voltage divider resistor value 500Ω / 1000Ω          [K
extern int32_t ecKoefA; //  Alfa value                                          [L
extern int32_t ecKoefB; //  Beta value                                          [M
extern int32_t ecKoefC; //  С-value                                             [N
extern int32_t ecKoefT; //  Ноль Koef Temperature                               [P

extern uint32_t ec_Hz; //  Частота ШиМа (в микросек, min 9, max 65535))         [Q
extern uint32_t skip_settings;

// Config.
extern uint32_t data[13];
