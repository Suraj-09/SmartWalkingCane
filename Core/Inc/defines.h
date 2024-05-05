#define SIM_APN                             ""  // Set valid SIM APN

#define SERVER_URL_FLAGS          ""
#define SERVER_URL_LOCATION       ""

#define PROCESS_BUFFER_SIZE       2000
#define RX_BUFFER_SIZE            400

#define LTEIOT9_INIT               0
#define LTEIOT9_NET_CONFIG         1
#define LTEIOT9_NET_CONN           2
#define LTEIOT9_HTTP_FLAGS_SETUP   3
#define LTEIOT9_HTTP_FLAGS_POST    4
#define LTEIOT9_GNSS_CONFIG        5
#define LTEIOT9_GNSS_DATA          6
#define LTEIOT9_HTTP_GNSS_SETUP    7
#define LTEIOT9_HTTP_GNSS_POST     8

#define INIT_ON                    0
#define INIT_SYSSTART              1
#define INIT_CHECK                 2

#define NET_CONFIG_CIMI            0
#define NET_CONFIG_SIM             1
#define NET_CONFIG_CREG            2

#define NET_CONN_CGATT             0
#define NET_CONN_CEREG             1
#define NET_CONN_CSQ               2

#define HTTP_FLAGS_SETUP_SICA_1    0
#define HTTP_FLAGS_SETUP_SICA      1
#define HTTP_FLAGS_SETUP_CGPADDR   2
#define HTTP_FLAGS_SETUP_SICA_0    3
#define HTTP_FLAGS_SETUP_SSECUA    4
#define HTTP_FLAGS_SETUP_DNS       5
#define HTTP_FLAGS_SETUP_SICA_ON   6
#define HTTP_FLAGS_SETUP_HTTP      7
#define HTTP_FLAGS_SETUP_CON       8
#define HTTP_FLAGS_SETUP_ADDR      9
#define HTTP_FLAGS_SETUP_HCPROP    10

#define HTTP_FLAGS_POST_POST       0
#define HTTP_FLAGS_POST_CONTENT    1
#define HTTP_FLAGS_POST_LEN        2
#define HTTP_FLAGS_POST_SISO       3
#define HTTP_FLAGS_POST_RESPONSE   4
#define HTTP_FLAGS_POST_SISC       5

#define HTTP_GNSS_SETUP_SICA_1     0
#define HTTP_GNSS_SETUP_SICA       1
#define HTTP_GNSS_SETUP_CGPADDR    2
#define HTTP_GNSS_SETUP_SICA_0     3
#define HTTP_GNSS_SETUP_SSECUA     4
#define HTTP_GNSS_SETUP_DNS        5
#define HTTP_GNSS_SETUP_SICA_ON    6
#define HTTP_GNSS_SETUP_HTTP       7
#define HTTP_GNSS_SETUP_CON        8
#define HTTP_GNSS_SETUP_ADDR       9
#define HTTP_GNSS_SETUP_HCPROP     10

#define HTTP_GNSS_POST_POST        0
#define HTTP_GNSS_POST_CONTENT     1
#define HTTP_GNSS_POST_LEN         2
#define HTTP_GNSS_POST_SISO        3
#define HTTP_GNSS_POST_RESPONSE    4
#define HTTP_GNSS_POST_SISC        5

#define GNSS_CONFIG_GPS            0
#define GNSS_CONFIG_NMEA_ON        1
#define GNSS_CONFIG_START_MODE     2
#define GNSS_WAIT_REBOOT           3

#define GNSS_DATA_POWER_UP         	0
#define GNSS_DATA_POWER_UP_CHECK   	1
#define GNSS_DATA_START_OUTPUT 		2
#define GNSS_DATA_PROCESS           3
#define GNSS_DATA_PARSE             4
#define GNSS_DATA_PARSE_COORDINATES 5
#define GNSS_DATA_STOP_OUTPUT		6
#define GNSS_DATA_POWER_DOWN       	7


#define GNNS_START_MODE_EN  "AT^SGPSC=\"Engine/StartMode\",0"
#define GNNS_START_GPS      "AT^SGPSC=\"Nmea/GPS\",\"on\""
#define GNSS_START_BEIDOU   "AT^SGPSC=\"Nmea/Beidou\",\"on\""
#define GNSS_START_GALILEO  "AT^SGPSC=\"Nmea/Galileo\",\"on\""
#define GNSS_START_GLONASS  "AT^SGPSC=\"Nmea/Glonass\",\"on\""
#define GNSS_POWER_UP       "AT^SGPSC=\"Engine\",\"3\""

#define GNSS_POWER_DOWN     "AT^SGPSC=\"Engine\",\"0\""
#define GNSS_STOP_OUT       "AT^SGPSC=\"Nmea/Output\",\"off\""
#define GNSS_START_OUT      "AT^SGPSC=\"Nmea/Output\",\"on\""
#define FIRST_TIME_GNSS     30
#define DEFAULT_WAIT        1000

#define US_1_TRIG_GPIO_Port     GPIOB
#define US_1_TRIG_Pin           GPIO_PIN_12

#define US_2_TRIG_GPIO_Port     GPIOB
#define US_2_TRIG_Pin           GPIO_PIN_13

#define US_1_ECHO_GPIO_Port     GPIOB
#define US_1_ECHO_Pin           GPIO_PIN_14

#define US_2_ECHO_GPIO_Port     GPIOB
#define US_2_ECHO_Pin           GPIO_PIN_15

#define VIBRATION_1_Port        GPIOB
#define VIBRATION_1_Pin         GPIO_PIN_8

#define VIBRATION_2_Port        GPIOB
#define VIBRATION_2_Pin         GPIO_PIN_7

#define BUZZER_Port             GPIOB
#define BUZZER_Pin              GPIO_PIN_9


#define LTEIOT9_TX_GPIO_Port    GPIOA
#define LTEIOT9_TX_Pin          GPIO_PIN_2

#define LTEIOT9_RX_GPIO_Port    GPIOA
#define LTEIOT9_RX_Pin          GPIO_PIN_3

#define LTEIOT9_CTS_GPIO_Port   GPIOA
#define LTEIOT9_CTS_Pin         GPIO_PIN_0

#define LTEIOT9_RTS_GPIO_Port   GPIOA
#define LTEIOT9_RTS_Pin         GPIO_PIN_1

#define LTEIOT9_ON_GPIO_Port    GPIOA
#define LTEIOT9_ON_Pin          GPIO_PIN_6

#define LTEIOT9_SMI_GPIO_Port   GPIOA
#define LTEIOT9_SMI_Pin         GPIO_PIN_5

#define LTEIOT9_CS_GPIO_Port    GPIOA
#define LTEIOT9_CS_Pin          GPIO_PIN_4

